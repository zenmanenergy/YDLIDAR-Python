#!/usr/bin/env python3
"""
Tests for the GS2 driver and geometry, including an end-to-end run against
simulated lidars. Run from the orangepi folder:

    python3 -m unittest discover tests -v

Needs pyserial. Linux/macOS only (the simulator uses pseudo-terminals).
"""

import json
import math
import os
import subprocess
import sys
import tempfile
import threading
import time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))

import geometry  # noqa: E402
import gs2  # noqa: E402
import lidar  # noqa: E402
from fake_gs2 import FAKE_CAL, FakeGS2  # noqa: E402

LENGTH = 0.84
WIDTH = 0.84
MOUNT_X = 0.38
MOUNT_Y = 0.38
MOUNT_YAW = 15.0


def wall_for_mount(gap_m, robot_yaw_deg, mx, my, myaw_deg):
    """
    Robot faces a wall `gap_m` past its front bumper, rotated robot_yaw_deg
    (CCW) away from square. Returns (distance_mm, normal_deg) of that wall
    as seen in a lidar's own frame.
    """
    normal_robot = math.radians(-robot_yaw_deg)
    center_dist = LENGTH / 2 + gap_m
    nx, ny = math.cos(normal_robot), math.sin(normal_robot)
    dist = center_dist - (nx * mx + ny * my)
    return dist * 1000.0, -robot_yaw_deg - myaw_deg


def make_config(path, left_port, right_port, left_yaw=MOUNT_YAW, right_yaw=-MOUNT_YAW):
    cfg = {
        "team": 0,
        "server": "127.0.0.1",
        "publish_hz": 50,
        "stale_s": 0.25,
        "robot": {"length_m": LENGTH, "width_m": WIDTH},
        "lidars": [
            {"name": "frontLeft", "port": left_port, "x_m": MOUNT_X, "y_m": MOUNT_Y, "yaw_deg": left_yaw},
            {"name": "frontRight", "port": right_port, "x_m": MOUNT_X, "y_m": -MOUNT_Y, "yaw_deg": right_yaw},
        ],
    }
    with open(path, "w") as f:
        json.dump(cfg, f)


def start_fakes(gap_m, robot_yaw_deg, left_actual_yaw=MOUNT_YAW, right_actual_yaw=-MOUNT_YAW, **kw):
    ld, ln = wall_for_mount(gap_m, robot_yaw_deg, MOUNT_X, MOUNT_Y, left_actual_yaw)
    rd, rn = wall_for_mount(gap_m, robot_yaw_deg, MOUNT_X, -MOUNT_Y, right_actual_yaw)
    return FakeGS2(ld, ln, **kw).start(), FakeGS2(rd, rn, **kw).start()


class ProtocolTests(unittest.TestCase):
    def test_packet_roundtrip(self):
        pkt = gs2.build_packet(0x61, b"\x01\x02\x03", addr=0x02)
        r = gs2.PacketReader()
        r.feed(b"\x00\xFF garbage" + pkt)
        self.assertEqual(list(r.packets()), [(0x02, 0x61, b"\x01\x02\x03")])

    def test_split_and_corrupt_packets(self):
        good = gs2.build_packet(0x63, bytes(range(10)))
        bad = bytearray(good)
        bad[10] ^= 0xFF
        r = gs2.PacketReader()
        stream = bytes(bad) + good + good
        out = []
        for i in range(0, len(stream), 3):  # arrive in small pieces
            r.feed(stream[i:i + 3])
            out.extend(r.packets())
        self.assertEqual(len(out), 2)
        self.assertEqual(r.checksum_errors, 1)

    def test_point_decoding(self):
        word = (77 << 9) | 200
        payload = b"\x00\x00" + bytes([word & 0xFF, word >> 8]) * 160
        pts, count = gs2.parse_scan_payload(payload, FAKE_CAL)
        self.assertEqual(count, 160)
        self.assertTrue(pts)
        self.assertTrue(all(p.intensity == 77 for p in pts))

    def test_field_of_view(self):
        angles = []
        for i in range(160):
            res = gs2.correct_point(150, i, FAKE_CAL)
            if res:
                angles.append(res[0])
        self.assertLess(min(angles), -40)
        self.assertGreater(max(angles), 40)

    def test_calibration_decoding(self):
        import struct
        cal = gs2.Calibration.from_payload(struct.pack("<HHHHb", 6000, 25000, 6100, 24000, -12))
        self.assertAlmostEqual(cal.k0, 0.6)
        self.assertAlmostEqual(cal.b0, 2.5)
        self.assertAlmostEqual(cal.bias, -1.2)


class GeometryTests(unittest.TestCase):
    def setUp(self):
        self.cfg = geometry.RobotConfig(0, "", LENGTH, WIDTH, [])

    def test_transform(self):
        m = geometry.LidarMount("a", "p", x_m=0.4, y_m=0.2, yaw_deg=90)
        (x, y), = geometry.to_robot_frame([gs2.Point(0.0, 100.0, 0)], m)
        self.assertAlmostEqual(x, 0.4)
        self.assertAlmostEqual(y, 0.3)

    def test_flip(self):
        m = geometry.LidarMount("a", "p", flip=True)
        (x, y), = geometry.to_robot_frame([gs2.Point(math.radians(30), 100.0, 0)], m)
        self.assertLess(y, 0)

    def test_wall_fit_angle_sign(self):
        # Robot rotated 10 deg CCW from square: the wall appears rotated CW
        a = math.radians(10)
        pts = [(0.6 + t * math.sin(a), t * math.cos(a)) for t in [i / 100 - 0.2 for i in range(41)]]
        fit, valid = geometry.fit_front_wall(pts, self.cfg)
        self.assertTrue(valid)
        self.assertAlmostEqual(fit.angle_deg, 10, places=6)

    def test_front_distance_ignores_sides(self):
        pts = [(0.55, 0.0), (0.45, 0.6)]
        self.assertAlmostEqual(geometry.front_distance(pts, self.cfg), 0.13)
        self.assertIsNone(geometry.front_distance([], self.cfg))


class EndToEndTests(unittest.TestCase):
    """Run the real readers against two simulated lidars."""

    def run_pipeline(self, gap_m, robot_yaw_deg, seconds=1.0, **kw):
        left, right = start_fakes(gap_m, robot_yaw_deg, **kw)
        try:
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, "config.json")
                make_config(path, left.path, right.path)
                cfg = geometry.load_config(path)
                lock = threading.Lock()
                stop = threading.Event()
                states, readers = lidar.build_readers(cfg, lock, stop)
                for r in readers:
                    r.start()
                time.sleep(seconds)
                per, merged, oldest = lidar.snapshot(states, lock, time.monotonic(), cfg.stale_s)
                stop.set()
                for r in readers:
                    r.join(timeout=3)
                return cfg, per, merged, readers
        finally:
            left.stop()
            right.stop()

    def test_square_to_wall(self):
        cfg, per, merged, _ = self.run_pipeline(0.15, 0.0)
        self.assertTrue(all(p[1] for p in per), per)
        self.assertGreater(len(merged), 100)
        front = geometry.front_distance(merged, cfg)
        self.assertAlmostEqual(front, 0.15, delta=0.004)
        wall, valid = geometry.fit_front_wall(merged, cfg)
        self.assertTrue(valid)
        self.assertAlmostEqual(wall.angle_deg, 0.0, delta=0.5)
        self.assertAlmostEqual(wall.distance_m - LENGTH / 2, 0.15, delta=0.004)
        for name, _f, _p, _ts, frames, fps, _c, _s in per:
            self.assertGreater(fps, 20, name)

    def test_rotated_robot(self):
        cfg, per, merged, _ = self.run_pipeline(0.12, 6.0)
        wall, valid = geometry.fit_front_wall(merged, cfg)
        self.assertTrue(valid)
        self.assertAlmostEqual(wall.angle_deg, 6.0, delta=0.5)

    def test_survives_corrupt_frames(self):
        cfg, per, merged, readers = self.run_pipeline(0.15, 0.0, corrupt_every=4)
        self.assertTrue(all(p[1] for p in per))
        self.assertGreater(sum(r.checksum_errors for r in readers), 0)

    def test_print_mode_subprocess(self):
        left, right = start_fakes(0.15, 0.0)
        try:
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, "config.json")
                make_config(path, left.path, right.path)
                p = subprocess.Popen([sys.executable, os.path.join(ROOT, "lidar.py"), "--config", path, "--print"],
                                     stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                time.sleep(2.0)
                p.terminate()
                out, _ = p.communicate(timeout=5)
        finally:
            left.stop()
            right.stop()
        self.assertEqual(p.returncode, 0, out)
        self.assertIn("frontLeft: ok", out)
        self.assertIn("frontRight: ok", out)

    def test_calibrate_finds_mount_error(self):
        # Config says 15 deg, but the left lidar is really mounted at 17.5
        left, right = start_fakes(0.15, 0.0, left_actual_yaw=17.5)
        try:
            with tempfile.TemporaryDirectory() as d:
                path = os.path.join(d, "config.json")
                make_config(path, left.path, right.path)
                p = subprocess.run([sys.executable, os.path.join(ROOT, "calibrate.py"), "--config", path,
                                    "--seconds", "1", "--distance-mm", "150", "--write"],
                                   capture_output=True, text=True, timeout=30)
                with open(path) as f:
                    saved = json.load(f)
        finally:
            left.stop()
            right.stop()
        self.assertEqual(p.returncode, 0, p.stdout + p.stderr)
        by_name = {e["name"]: e for e in saved["lidars"]}
        self.assertAlmostEqual(by_name["frontLeft"]["yaw_deg"], 17.5, delta=0.5)
        self.assertAlmostEqual(by_name["frontRight"]["yaw_deg"], -15.0, delta=0.5)
        self.assertAlmostEqual(by_name["frontRight"]["x_m"], MOUNT_X, delta=0.005)


if __name__ == "__main__":
    unittest.main()

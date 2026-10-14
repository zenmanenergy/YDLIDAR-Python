#!/usr/bin/env python3
"""
Simulated GS2 lidar on a virtual serial port, for testing without hardware.

    python3 tools/fake_gs2.py --wall-mm 180 --wall-angle-deg -15

It prints a device path like /dev/pts/5. Put that path in a test config's
"port" field and run lidar.py --print or calibrate.py against it.

The scene is one flat wall. --wall-mm is the wall's perpendicular distance
from the lidar, and --wall-angle-deg is the direction of the wall's normal
in the lidar's own frame (CCW positive). For a lidar mounted at yaw 15 deg
on a robot facing a wall squarely, use --wall-angle-deg -15.

The calibration values are made up but realistic enough to give a ~100 deg
field of view. Linux/macOS only (uses a pseudo-terminal).
"""

import argparse
import math
import os
import random
import struct
import sys
import threading
import time
import tty

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import gs2  # noqa: E402

# Stored the same way the real device sends them (value * 10000)
FAKE_PARAMS = struct.pack("<HHHHb", 118, 5320, 118, 4260, 0)
FAKE_CAL = gs2.Calibration.from_payload(FAKE_PARAMS)
FRAME_HZ = 27


def raw_for_index(i, wall_mm, normal_rad, noise_mm):
    """Find the raw 9-bit reading that the real sensor would report at index i."""
    raw = wall_mm
    for _ in range(12):
        res = gs2.correct_point(max(1, int(round(raw))), i, FAKE_CAL)
        if res is None:
            return 0
        theta = math.radians(res[0])
        c = math.cos(theta - normal_rad)
        if c < 0.15:
            return 0  # grazing angle or facing away: no return
        target = wall_mm / c
        raw += target - res[1]
    raw += random.gauss(0, noise_mm)
    raw = int(round(raw))
    if raw <= 0 or raw > 300:
        return 0
    return raw


def make_frame(addr, wall_mm, normal_rad, noise_mm):
    words = []
    for i in range(gs2.POINTS_PER_FRAME):
        raw = raw_for_index(i, wall_mm, normal_rad, noise_mm)
        intensity = 60 if raw else 0
        words.append((intensity << 9) | (raw & 0x1FF))
    payload = struct.pack("<H", 0) + struct.pack("<%dH" % len(words), *words)
    return gs2.build_packet(gs2.CMD_SCAN, payload, addr)


class FakeGS2:
    def __init__(self, wall_mm, wall_angle_deg, noise_mm=1.0, addr=0x01, corrupt_every=0):
        self.master, slave = os.openpty()
        tty.setraw(slave)
        self.path = os.ttyname(slave)
        self._slave = slave
        self.wall_mm = wall_mm
        self.normal = math.radians(wall_angle_deg)
        self.noise = noise_mm
        self.addr = addr
        self.corrupt_every = corrupt_every
        self.scanning = False
        self.frames_sent = 0
        self.lock = threading.Lock()
        self.stop_event = threading.Event()

    def _send(self, data):
        with self.lock:
            os.write(self.master, data)

    def _command_loop(self):
        reader = gs2.PacketReader()
        while not self.stop_event.is_set():
            try:
                data = os.read(self.master, 256)
            except OSError:
                return
            reader.feed(data)
            for _addr, ptype, _payload in reader.packets():
                if ptype == gs2.CMD_STOP:
                    self.scanning = False
                    self._send(gs2.build_packet(gs2.CMD_STOP, b"", self.addr))
                elif ptype == gs2.CMD_GET_ADDRESS:
                    self._send(gs2.build_packet(gs2.CMD_GET_ADDRESS, bytes([self.addr]), self.addr))
                elif ptype == gs2.CMD_GET_PARAMETER:
                    self._send(gs2.build_packet(gs2.CMD_GET_PARAMETER, FAKE_PARAMS, self.addr))
                elif ptype == gs2.CMD_SCAN:
                    self._send(gs2.build_packet(gs2.CMD_SCAN, b"", self.addr))
                    self.scanning = True

    def _stream_loop(self):
        period = 1.0 / FRAME_HZ
        while not self.stop_event.is_set():
            if self.scanning:
                frame = bytearray(make_frame(self.addr, self.wall_mm, self.normal, self.noise))
                self.frames_sent += 1
                if self.corrupt_every and self.frames_sent % self.corrupt_every == 0:
                    frame[100] ^= 0xFF  # damage a byte to exercise checksum handling
                self._send(bytes(frame))
            time.sleep(period)

    def start(self):
        threading.Thread(target=self._command_loop, daemon=True).start()
        threading.Thread(target=self._stream_loop, daemon=True).start()
        return self

    def stop(self):
        self.stop_event.set()


def main():
    ap = argparse.ArgumentParser(description="Simulated GS2 on a virtual serial port")
    ap.add_argument("--wall-mm", type=float, default=180.0)
    ap.add_argument("--wall-angle-deg", type=float, default=0.0)
    ap.add_argument("--noise-mm", type=float, default=1.0)
    args = ap.parse_args()
    dev = FakeGS2(args.wall_mm, args.wall_angle_deg, args.noise_mm).start()
    print(dev.path, flush=True)
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        dev.stop()


if __name__ == "__main__":
    main()

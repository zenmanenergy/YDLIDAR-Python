#!/usr/bin/env python3
"""
Reads every GS2 lidar listed in config.json, converts their points into
robot coordinates, merges them, and publishes the results to NetworkTables
under /lidar.

Run as a service (see lidar.service), or by hand for testing:

    python3 lidar.py --config config.json            publish to the roboRIO
    python3 lidar.py --config config.json --print    no NetworkTables, print to console
"""

import argparse
import logging
import signal
import sys
import threading
import time
from dataclasses import dataclass, field

import serial

import geometry
import gs2

log = logging.getLogger("lidar")


@dataclass
class LidarState:
    mount: geometry.LidarMount
    points: list = field(default_factory=list)  # (x, y) robot frame, meters
    timestamp: float = 0.0                       # time.monotonic() of the frame
    frames: int = 0
    fps: float = 0.0
    connected: bool = False
    status: str = "starting"


class PortReader(threading.Thread):
    """Owns one serial port and feeds every lidar on it (1, or up to 3 cascaded)."""

    def __init__(self, port, states, lock, stop_event):
        super().__init__(daemon=True, name="gs2:" + port)
        self.port = port
        self.by_module = {s.mount.module: s for s in states}
        self.lock = lock
        self.stop_event = stop_event
        self.checksum_errors = 0
        self._dev = None

    def _set_all(self, connected, status):
        with self.lock:
            for s in self.by_module.values():
                s.connected = connected
                s.status = status

    def run(self):
        last_error = None
        while not self.stop_event.is_set():
            dev = gs2.GS2Port(self.port)
            self._dev = dev
            try:
                dev.open()
                dev.start()
                log.info("%s: scanning, %d unit(s) on this port", self.port, dev.module_count)
                last_error = None
                self._set_all(True, "ok")
                for frame in dev.frames():
                    if self.stop_event.is_set():
                        break
                    self.checksum_errors = dev.checksum_errors
                    state = self.by_module.get(frame.module)
                    if state is None:
                        continue
                    pts = geometry.to_robot_frame(frame.points, state.mount)
                    with self.lock:
                        if state.timestamp:
                            dt = frame.timestamp - state.timestamp
                            if dt > 0:
                                # Smoothed frames per second
                                state.fps = 0.9 * state.fps + 0.1 * (1.0 / dt) if state.fps else 1.0 / dt
                        state.points = pts
                        state.timestamp = frame.timestamp
                        state.frames += 1
            except (serial.SerialException, gs2.GS2Error, OSError) as e:
                msg = str(e)
                if msg != last_error:
                    log.warning("%s: %s (retrying)", self.port, msg)
                    last_error = msg
                self._set_all(False, msg)
            finally:
                dev.close()
            self.stop_event.wait(1.0)


def build_readers(cfg, lock, stop_event):
    states = {}
    by_port = {}
    for mount in cfg.lidars:
        if not mount.enabled:
            continue
        st = LidarState(mount)
        states[mount.name] = st
        by_port.setdefault(mount.port, []).append(st)
    readers = [PortReader(port, sts, lock, stop_event) for port, sts in by_port.items()]
    return states, readers


def snapshot(states, lock, now, stale_s):
    """Copy out the current state. Returns (per-lidar list, merged points, oldest fresh timestamp)."""
    with lock:
        per = []
        merged = []
        oldest = None
        for st in states.values():
            fresh = st.connected and st.timestamp > 0 and (now - st.timestamp) <= stale_s
            per.append((st.mount.name, fresh, list(st.points), st.timestamp, st.frames, st.fps, st.connected, st.status))
            if fresh:
                merged.extend(st.points)
                oldest = st.timestamp if oldest is None else min(oldest, st.timestamp)
    return per, merged, oldest


def flatten(points):
    out = []
    for x, y in points:
        out.append(x)
        out.append(y)
    return out


class Publisher:
    """All NetworkTables output lives here."""

    def __init__(self, cfg, names):
        import ntcore
        self.half_length = cfg.length_m / 2.0
        self.inst = ntcore.NetworkTableInstance.getDefault()
        # WPILib 2026 calls this startClient4; 2027 renamed it to startClient
        start = getattr(self.inst, "startClient4", None) or self.inst.startClient
        start("lidar")
        if cfg.server:
            self.inst.setServer(cfg.server)
        else:
            self.inst.setServerTeam(cfg.team)

        t = self.inst.getTable("lidar")
        self.heartbeat = t.getIntegerTopic("heartbeat").publish()
        self.connected = t.getBooleanTopic("connected").publish()
        self.latency = t.getDoubleTopic("latencyMs").publish()
        self.front = t.getDoubleTopic("frontDistance").publish()
        self.wall_valid = t.getBooleanTopic("wall/valid").publish()
        self.wall_dist = t.getDoubleTopic("wall/distance").publish()
        self.wall_angle = t.getDoubleTopic("wall/angleDeg").publish()
        self.wall_rms = t.getDoubleTopic("wall/rms").publish()
        self.merged = t.getDoubleArrayTopic("merged/points").publish()
        self.robot_length = t.getDoubleTopic("robot/length").publish()
        self.robot_width = t.getDoubleTopic("robot/width").publish()
        self.robot_length.set(cfg.length_m)
        self.robot_width.set(cfg.width_m)

        # Optional: Translation2d[] structs so AdvantageScope can draw the
        # points on its 2D field view. Needs robotpy-wpimath installed.
        self.merged_struct = None
        try:
            from wpimath.geometry import Translation2d
            self._T2d = Translation2d
            self.merged_struct = t.getStructArrayTopic("merged/translations", Translation2d).publish()
        except Exception as e:
            log.info("AdvantageScope struct output disabled (%s)", e)

        self.per = {}
        for name in names:
            sub = t.getSubTable(name)
            self.per[name] = {
                "connected": sub.getBooleanTopic("connected").publish(),
                "fps": sub.getDoubleTopic("fps").publish(),
                "points": sub.getDoubleArrayTopic("points").publish(),
                "count": sub.getIntegerTopic("pointCount").publish(),
                "status": sub.getStringTopic("status").publish(),
                "errors": sub.getIntegerTopic("checksumErrors").publish(),
            }

    def publish(self, beat, per, merged, latency_ms, front, wall, wall_valid, errors_by_name):
        self.heartbeat.set(beat)
        self.connected.set(any(p[1] for p in per))
        self.latency.set(latency_ms)
        self.front.set(front if front is not None else -1.0)
        self.wall_valid.set(bool(wall_valid))
        if wall is not None:
            # Measured from the front bumper, like frontDistance
            self.wall_dist.set(wall.distance_m - self.half_length)
            self.wall_angle.set(wall.angle_deg)
            self.wall_rms.set(wall.rms_m)
        self.merged.set(flatten(merged))
        if self.merged_struct is not None:
            self.merged_struct.set([self._T2d(x, y) for x, y in merged])
        for name, fresh, pts, _ts, _frames, fps, connected, status in per:
            p = self.per[name]
            p["connected"].set(fresh)
            p["fps"].set(fps)
            p["points"].set(flatten(pts) if fresh else [])
            p["count"].set(len(pts) if fresh else 0)
            p["status"].set(status)
            p["errors"].set(errors_by_name.get(name, 0))

    def close(self):
        self.inst.stopClient()


def main():
    ap = argparse.ArgumentParser(description="GS2 lidar to NetworkTables")
    ap.add_argument("--config", default="/opt/lidar/config.json")
    ap.add_argument("--print", action="store_true", help="print to console instead of NetworkTables")
    args = ap.parse_args()

    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s",
                        stream=sys.stdout)

    cfg = geometry.load_config(args.config)
    lock = threading.Lock()
    stop_event = threading.Event()
    states, readers = build_readers(cfg, lock, stop_event)
    if not readers:
        log.error("no enabled lidars in %s", args.config)
        return 1

    pub = None if args.print else Publisher(cfg, list(states.keys()))
    if pub:
        target = cfg.server or "team %d" % cfg.team
        log.info("publishing to NetworkTables (%s)", target)

    def handle_signal(_sig, _frame):
        stop_event.set()
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGINT, handle_signal)

    for r in readers:
        r.start()

    period = 1.0 / cfg.publish_hz
    beat = 0
    last_print = 0.0
    next_tick = time.monotonic()
    while not stop_event.is_set():
        now = time.monotonic()
        per, merged, oldest = snapshot(states, lock, now, cfg.stale_s)
        front = geometry.front_distance(merged, cfg)
        wall, wall_valid = geometry.fit_front_wall(merged, cfg)
        latency_ms = (now - oldest) * 1000.0 if oldest is not None else -1.0
        errors_by_name = {}
        for r in readers:
            for st in r.by_module.values():
                errors_by_name[st.mount.name] = r.checksum_errors
        beat += 1

        if pub:
            pub.publish(beat, per, merged, latency_ms, front, wall, wall_valid, errors_by_name)
        elif now - last_print >= 0.5:
            last_print = now
            parts = []
            for name, fresh, pts, _ts, _f, fps, _c, status in per:
                parts.append("%s: %s %3d pts %4.1f Hz" % (name, "ok " if fresh else "-- ", len(pts), fps) +
                             ("" if fresh else " (%s)" % status))
            front_txt = "%5.0f mm" % (front * 1000) if front is not None else "  none "
            wall_txt = ("wall %4.0f mm %+5.1f deg" % ((wall.distance_m - cfg.length_m / 2) * 1000, wall.angle_deg)
                        if wall_valid else "wall --")
            print("front %s | %s | %s" % (front_txt, wall_txt, " | ".join(parts)), flush=True)

        next_tick += period
        delay = next_tick - time.monotonic()
        if delay > 0:
            stop_event.wait(delay)
        else:
            next_tick = time.monotonic()

    for r in readers:
        r.join(timeout=2.0)
    if pub:
        pub.close()
    log.info("stopped")
    return 0


if __name__ == "__main__":
    sys.exit(main())

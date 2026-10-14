#!/usr/bin/env python3
"""
Mount calibration against a flat wall.

1. Stop the service so the serial ports are free:   sudo systemctl stop lidar
2. Face the robot squarely toward a flat wall, about 100-200 mm away, so the
   wall is inside every lidar's range. Use a carpenter's square or two equal
   tape measurements at the bumper corners to get it square. Don't touch the
   wall: the GS2 can't measure very close objects.
3. Run:   python3 calibrate.py --config /opt/lidar/config.json
   Optionally add --distance-mm N, the measured gap between the front bumper
   and the wall, to also check the x positions.
4. Add --write to save the corrected yaw (and x) values into the config.
5. Start the service again:   sudo systemctl start lidar

Each lidar is fitted on its own. If the robot is square to the wall, every
lidar should see the wall at 0 degrees. Any angle it reports is mounting
error, and the fix is to add it to that lidar's yaw_deg.
"""

import argparse
import json
import sys
import time

import geometry
import gs2


def collect(mount, seconds):
    """Read one lidar for a few seconds; return all its points in robot coordinates."""
    dev = gs2.GS2Port(mount.port)
    pts = []
    frames = 0
    try:
        dev.open()
        dev.start()
        end = time.monotonic() + seconds
        for frame in dev.frames():
            if frame.module == mount.module:
                pts.extend(geometry.to_robot_frame(frame.points, mount))
                frames += 1
            if time.monotonic() > end:
                break
    finally:
        dev.close()
    return pts, frames


def main():
    ap = argparse.ArgumentParser(description="Calibrate lidar mount yaw against a flat wall")
    ap.add_argument("--config", default="/opt/lidar/config.json")
    ap.add_argument("--seconds", type=float, default=2.0, help="how long to sample each lidar")
    ap.add_argument("--distance-mm", type=float, default=None,
                    help="measured gap from front bumper to wall, to check x_m")
    ap.add_argument("--write", action="store_true", help="save corrected values into the config")
    args = ap.parse_args()

    cfg = geometry.load_config(args.config)
    with open(args.config) as f:
        raw = json.load(f)

    changes = {}
    for mount in cfg.lidars:
        if not mount.enabled:
            continue
        print("\n%s (%s)" % (mount.name, mount.port))
        try:
            pts, frames = collect(mount, args.seconds)
        except Exception as e:
            print("  could not read: %s" % e)
            continue
        fit, valid = geometry.fit_front_wall(pts, cfg, side_margin_m=1.0, min_points=20)
        if fit is None:
            print("  no points found ahead of the robot. Is the wall in range?")
            continue
        print("  %d frames, %d points, fit error %.1f mm" % (frames, fit.count, fit.rms_m * 1000))
        if not valid:
            print("  points do not form a clean straight line. Check the wall is flat and nothing else is in view.")
            continue

        # A lidar turned further CCW than the config says makes the wall look
        # like the robot itself turned CCW, so the correction is to add.
        new_yaw = mount.yaw_deg + fit.angle_deg
        print("  wall angle seen: %+.2f deg  ->  yaw_deg %.2f becomes %.2f" % (fit.angle_deg, mount.yaw_deg, new_yaw))
        change = {"yaw_deg": round(new_yaw, 2)}

        if args.distance_mm is not None:
            measured = (fit.distance_m - cfg.length_m / 2.0) * 1000.0
            error_m = (measured - args.distance_mm) / 1000.0
            new_x = mount.x_m - error_m
            print("  bumper gap measured %.0f mm vs actual %.0f mm  ->  x_m %.3f becomes %.3f"
                  % (measured, args.distance_mm, mount.x_m, new_x))
            change["x_m"] = round(new_x, 4)
        changes[mount.name] = change

    if not changes:
        print("\nNothing to update.")
        return 1

    if args.write:
        for entry in raw["lidars"]:
            if entry["name"] in changes:
                entry.update(changes[entry["name"]])
        with open(args.config, "w") as f:
            json.dump(raw, f, indent=2)
            f.write("\n")
        print("\nSaved to %s." % args.config)
        print("Copy these values into config.json in the team repo too, so the next deploy keeps them.")
    else:
        print("\nRun again with --write to save these values.")
    return 0


if __name__ == "__main__":
    sys.exit(main())

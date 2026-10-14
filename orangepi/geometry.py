"""
Robot geometry: config loading, lidar-to-robot transforms, and the derived
measurements (front distance, wall fit).

Coordinates follow the WPILib convention:
    origin = center of the robot
    +x = forward, +y = left
    yaw is counterclockwise positive
    distances in meters
"""

import json
import math
from dataclasses import dataclass


@dataclass
class LidarMount:
    name: str
    port: str
    module: int = 0          # position in a cascade chain (0 if one lidar per port)
    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0
    flip: bool = False       # set true if left and right come out mirrored
    enabled: bool = True


@dataclass
class RobotConfig:
    team: int
    server: str              # optional NetworkTables server override, "" to use team number
    length_m: float          # front to back, including bumpers
    width_m: float           # side to side, including bumpers
    lidars: list
    publish_hz: float = 50.0
    stale_s: float = 0.25    # ignore a lidar's frame if it is older than this


def load_config(path):
    with open(path) as f:
        raw = json.load(f)
    robot = raw["robot"]
    mounts = []
    for entry in raw["lidars"]:
        mounts.append(LidarMount(
            name=entry["name"],
            port=entry["port"],
            module=int(entry.get("module", 0)),
            x_m=float(entry["x_m"]),
            y_m=float(entry["y_m"]),
            yaw_deg=float(entry["yaw_deg"]),
            flip=bool(entry.get("flip", False)),
            enabled=bool(entry.get("enabled", True)),
        ))
    names = [m.name for m in mounts]
    if len(set(names)) != len(names):
        raise ValueError("lidar names in config must be unique")
    return RobotConfig(
        team=int(raw.get("team", 0)),
        server=str(raw.get("server", "")),
        length_m=float(robot["length_m"]),
        width_m=float(robot["width_m"]),
        lidars=mounts,
        publish_hz=float(raw.get("publish_hz", 50.0)),
        stale_s=float(raw.get("stale_s", 0.25)),
    )


def to_robot_frame(points, mount):
    """
    Lidar points (angle in radians, distance in mm, lidar frame) ->
    list of (x, y) in meters, robot frame.
    """
    yaw = math.radians(mount.yaw_deg)
    c, s = math.cos(yaw), math.sin(yaw)
    sign = -1.0 if mount.flip else 1.0
    out = []
    for p in points:
        d = p.distance / 1000.0
        lx = d * math.cos(p.angle)
        ly = sign * d * math.sin(p.angle)
        out.append((mount.x_m + lx * c - ly * s, mount.y_m + lx * s + ly * c))
    return out


def front_distance(points, cfg, side_margin_m=0.0):
    """
    Distance from the front bumper to the closest point directly ahead
    (within the robot's width). Returns None if nothing is ahead.
    """
    front = cfg.length_m / 2.0
    half_w = cfg.width_m / 2.0 + side_margin_m
    best = None
    for x, y in points:
        if abs(y) <= half_w and x >= front - 0.01:
            d = x - front
            if best is None or d < best:
                best = d
    if best is None:
        return None
    return max(best, 0.0)


@dataclass
class LineFit:
    angle_deg: float      # robot yaw relative to facing the wall squarely, CCW positive
    distance_m: float     # perpendicular distance from robot center to the wall
    rms_m: float          # how well the points fit a straight line
    count: int


def fit_line(points):
    """
    Total-least-squares line fit (works for any orientation).
    Returns a LineFit, or None with fewer than 2 points.

    angle_deg is the robot's rotation away from square: the wall direction
    in robot coordinates is (sin a, cos a). Turn by -angle_deg to square up.
    """
    n = len(points)
    if n < 2:
        return None
    mx = sum(p[0] for p in points) / n
    my = sum(p[1] for p in points) / n
    sxx = sum((p[0] - mx) ** 2 for p in points)
    syy = sum((p[1] - my) ** 2 for p in points)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in points)
    # Principal direction of the point spread
    phi = 0.5 * math.atan2(2 * sxy, sxx - syy)
    dx, dy = math.cos(phi), math.sin(phi)
    if dy < 0:
        dx, dy = -dx, -dy
    angle = math.degrees(math.atan2(dx, dy))
    # Perpendicular distance from origin, and residuals
    nx, ny = dy, -dx
    distance = abs(mx * nx + my * ny)
    rms = math.sqrt(sum(((p[0] - mx) * nx + (p[1] - my) * ny) ** 2 for p in points) / n)
    return LineFit(angle, distance, rms, n)


def fit_front_wall(points, cfg, side_margin_m=0.10, min_points=10, max_rms_m=0.015):
    """
    Fit a wall in front of the robot. Returns (LineFit, valid).
    valid is False when there are too few points or they are not a straight line.
    """
    front = cfg.length_m / 2.0
    half_w = cfg.width_m / 2.0 + side_margin_m
    ahead = [(x, y) for x, y in points if x >= front - 0.01 and abs(y) <= half_w]
    fit = fit_line(ahead)
    if fit is None:
        return None, False
    valid = fit.count >= min_points and fit.rms_m <= max_rms_m and abs(fit.angle_deg) < 60
    return fit, valid

"""
Reads the lidar data published by lidar.py on the Orange Pi (NetworkTables table "/lidar").

All coordinates are robot-relative, WPILib convention: origin at robot center, +x forward,
+y left, counterclockwise-positive angles, meters. Distances called "from the bumper" are
measured from the front bumper face, using the robot length set in config.json on the Pi.

Every getter returns None when the data is stale, so if the Orange Pi script stops or a lidar
unplugs, commands see "no data" instead of an old number.
"""

import math
import typing
from dataclasses import dataclass

import commands2
import ntcore
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d

# Data older than this is treated as missing
STALE_SECONDS = 0.25


@dataclass(frozen=True)
class Wall:
    """A flat surface in front of the robot."""

    distance: float
    """Perpendicular distance from the front bumper to the wall, meters."""

    angle: Rotation2d
    """How far the robot is turned away from facing the wall squarely, counterclockwise
    positive. Turn by the negative of this to square up."""


def closest_point_along_ray(
    points: typing.Sequence[Translation2d],
    origin: Translation2d,
    direction: Rotation2d,
    half_width: float,
) -> typing.Optional[Translation2d]:
    """
    The point nearest to `origin` inside a corridor of +/- half_width meters around a ray.
    Only points in front of the origin (along the ray) count.
    """
    cos, sin = direction.cos(), direction.sin()
    best_along = math.inf
    best = None
    for p in points:
        vx = p.x - origin.x
        vy = p.y - origin.y
        along = vx * cos + vy * sin
        across = -vx * sin + vy * cos
        if along > 0 and abs(across) <= half_width and along < best_along:
            best_along = along
            best = p
    return best


def to_field(points: typing.Sequence[Translation2d], robot_pose: Pose2d) -> typing.List[Translation2d]:
    """Robot-relative points to field coordinates."""
    rot = robot_pose.rotation()
    origin = robot_pose.translation()
    return [p.rotateBy(rot) + origin for p in points]


def unflatten(flat: typing.Sequence[float]) -> typing.List[Translation2d]:
    """[x0, y0, x1, y1, ...] -> list of Translation2d."""
    return [Translation2d(flat[i], flat[i + 1]) for i in range(0, len(flat) - 1, 2)]


class LidarSubsystem(commands2.Subsystem):
    def __init__(self, pose_supplier: typing.Optional[typing.Callable[[], Pose2d]] = None):
        """
        :param pose_supplier: returns the robot's field pose (for example from a pose estimator
            fed by PhotonVision). When given, lidar points are also published in field
            coordinates to /Lidar/FieldPoints for AdvantageScope.
        """
        super().__init__()
        self.pose_supplier = pose_supplier

        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("lidar")
        self._heartbeat = table.getIntegerTopic("heartbeat").subscribe(0)
        self._connected = table.getBooleanTopic("connected").subscribe(False)
        self._latency_ms = table.getDoubleTopic("latencyMs").subscribe(-1.0)
        self._front = table.getDoubleTopic("frontDistance").subscribe(-1.0)
        self._wall_valid = table.getBooleanTopic("wall/valid").subscribe(False)
        self._wall_distance = table.getDoubleTopic("wall/distance").subscribe(0.0)
        self._wall_angle = table.getDoubleTopic("wall/angleDeg").subscribe(0.0)
        self._merged = table.getDoubleArrayTopic("merged/points").subscribe([])

        self._field_points_pub = None
        if pose_supplier is not None:
            self._field_points_pub = inst.getStructArrayTopic(
                "/Lidar/FieldPoints", Translation2d
            ).publish()

        self._last_beat = None
        self._last_beat_change = -math.inf
        self._fresh = False
        self._points: typing.List[Translation2d] = []
        self._points_timestamp = 0.0

    def periodic(self) -> None:
        now = wpilib.Timer.getFPGATimestamp()

        # The Pi bumps the heartbeat every update. If it stops changing, the script is not running.
        beat = self._heartbeat.get()
        if beat != self._last_beat:
            self._last_beat = beat
            self._last_beat_change = now
        self._fresh = (now - self._last_beat_change) < STALE_SECONDS and self._connected.get()

        if self._fresh:
            sample = self._merged.getAtomic()
            self._points = unflatten(sample.value)
            # NetworkTables timestamps on the robot use the FPGA clock, in microseconds
            self._points_timestamp = sample.time / 1e6 - max(self._latency_ms.get(), 0.0) / 1000.0
        else:
            self._points = []

        front = self.getFrontDistance()
        wpilib.SmartDashboard.putBoolean("Lidar/Connected", self._fresh)
        wpilib.SmartDashboard.putNumber("Lidar/FrontDistance", -1.0 if front is None else front)
        wpilib.SmartDashboard.putNumber("Lidar/PointCount", len(self._points))

        if self._field_points_pub is not None:
            pose = self.pose_supplier()
            self._field_points_pub.set([] if pose is None else to_field(self._points, pose))

    def isConnected(self) -> bool:
        """True when lidar data is arriving and at least one lidar is working."""
        return self._fresh

    def getFrontDistance(self) -> typing.Optional[float]:
        """
        Distance from the front bumper to the closest object directly ahead (within the
        robot's width), meters. None if nothing is in range or the data is stale.
        """
        if not self._fresh:
            return None
        d = self._front.get()
        return None if d < 0 else d

    def getWall(self) -> typing.Optional[Wall]:
        """The flat surface in front of the robot, if the points ahead form a clean line."""
        if not self._fresh or not self._wall_valid.get():
            return None
        return Wall(self._wall_distance.get(), Rotation2d.fromDegrees(self._wall_angle.get()))

    def getPoints(self) -> typing.List[Translation2d]:
        """All current lidar points, robot-relative, meters. Empty if stale."""
        return self._points

    def getPointsTimestamp(self) -> float:
        """FPGA time (seconds) when the current points were measured, for latency compensation."""
        return self._points_timestamp

    def getClosestPointAlongRay(
        self, origin: Translation2d, direction: Rotation2d, half_width: float
    ) -> typing.Optional[Translation2d]:
        """
        The closest lidar point near a ray from any spot on the robot. Useful for asking
        "what is in that direction?", for example from a camera's position toward a game piece.
        """
        return closest_point_along_ray(self._points, origin, direction, half_width)

    def getDistanceAlongRay(
        self, origin: Translation2d, direction: Rotation2d, half_width: float
    ) -> typing.Optional[float]:
        """Distance from origin to the closest point near the ray, meters, or None."""
        p = self.getClosestPointAlongRay(origin, direction, half_width)
        if p is None:
            return None
        return (p.x - origin.x) * direction.cos() + (p.y - origin.y) * direction.sin()

    def getDistanceAtBearing(self, bearing: Rotation2d, half_width: float) -> typing.Optional[float]:
        """Same as getDistanceAlongRay, from the robot's center."""
        return self.getDistanceAlongRay(Translation2d(0, 0), bearing, half_width)

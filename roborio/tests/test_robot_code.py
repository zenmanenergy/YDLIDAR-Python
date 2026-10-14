#!/usr/bin/env python3
"""
Tests for the roboRIO lidar code. Run from the roborio folder:

    python3 -m unittest discover tests -v

These run anywhere, without RobotPy: tests/fakes holds small stand-ins for wpilib, ntcore,
commands2 and wpimath that use the same names as RobotPy 2026, with real geometry math. They
check the logic (signs, handoffs, stopping) by simulating a robot driving each command to
completion. They don't check RobotPy itself, so still test on the real robot.
"""

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "fakes"))
sys.path.insert(0, os.path.dirname(HERE))

import ntcore  # noqa: E402  (the stand-in)
import wpilib  # noqa: E402
from wpimath.geometry import Pose2d, Rotation2d, Transform2d, Translation2d  # noqa: E402

from commands.drive_to_wall_distance import DriveToWallDistance  # noqa: E402
from commands.lidar_intake_assist import LidarIntakeAssist  # noqa: E402
from commands.square_to_wall import SquareToWall  # noqa: E402
from subsystems.lidar_subsystem import (  # noqa: E402
    LidarSubsystem,
    closest_point_along_ray,
    to_field,
    unflatten,
)

LENGTH = 0.84
DT = 0.02


def flatten(points):
    out = []
    for p in points:
        out += [p.x, p.y]
    return out


class FakeDrive:
    """Integrates robot-relative ChassisSpeeds into a field pose."""

    def __init__(self, x=0.0, y=0.0, heading_deg=0.0):
        self.x, self.y, self.h = x, y, math.radians(heading_deg)
        self.last = None

    def __call__(self, speeds):
        self.last = speeds

    def step(self):
        s = self.last
        if s is None:
            return
        self.h += s.omega * DT
        self.x += (s.vx * math.cos(self.h) - s.vy * math.sin(self.h)) * DT
        self.y += (s.vx * math.sin(self.h) + s.vy * math.cos(self.h)) * DT

    def to_robot(self, fx, fy):
        dx, dy = fx - self.x, fy - self.y
        c, s = math.cos(-self.h), math.sin(-self.h)
        return Translation2d(dx * c - dy * s, dx * s + dy * c)


class World:
    """Plays the role of lidar.py: writes what the Pi would publish for the current pose."""

    def __init__(self):
        self.beat = 0

    def publish(self, front=None, wall=None, points=()):
        self.beat += 1
        ntcore.STORE["/lidar/heartbeat"] = self.beat
        ntcore.STORE["/lidar/connected"] = True
        ntcore.STORE["/lidar/latencyMs"] = 30.0
        ntcore.STORE["/lidar/frontDistance"] = -1.0 if front is None else front
        ntcore.STORE["/lidar/wall/valid"] = wall is not None
        if wall is not None:
            ntcore.STORE["/lidar/wall/distance"] = wall[0]
            ntcore.STORE["/lidar/wall/angleDeg"] = wall[1]
        ntcore.STORE["/lidar/merged/points"] = flatten(points)
        ntcore.TIMES["/lidar/merged/points"] = int(wpilib.Timer.now * 1e6)


def reset():
    ntcore.STORE.clear()
    ntcore.TIMES.clear()
    ntcore.PUBLISHED.clear()
    wpilib.Timer.now = 100.0


def run(command, lidar, drive, world_update, max_seconds=10.0):
    """Mini scheduler: subsystem periodic, then command, then physics."""
    command.initialize()
    t = 0.0
    while t < max_seconds:
        world_update()
        lidar.periodic()
        command.execute()
        drive.step()
        wpilib.Timer.now += DT
        t += DT
        if command.isFinished():
            command.end(False)
            return t
    command.end(True)
    return None


class HelperTests(unittest.TestCase):
    def test_unflatten(self):
        pts = unflatten([1.0, 2.0, 3.0, 4.0, 5.0])  # trailing odd value ignored
        self.assertEqual([(p.x, p.y) for p in pts], [(1.0, 2.0), (3.0, 4.0)])

    def test_to_field(self):
        pose = Pose2d(Translation2d(2.0, 1.0), Rotation2d.fromDegrees(90))
        (p,) = to_field([Translation2d(1.0, 0.0)], pose)
        self.assertAlmostEqual(p.x, 2.0)
        self.assertAlmostEqual(p.y, 2.0)

    def test_ray_search_from_offset_origin(self):
        pts = [Translation2d(0.6, 0.30), Translation2d(0.5, 0.0), Translation2d(0.9, 0.31)]
        origin = Translation2d(0.3, 0.3)
        p = closest_point_along_ray(pts, origin, Rotation2d(0), 0.05)
        self.assertAlmostEqual(p.x, 0.6)
        # Points behind the origin never count
        self.assertIsNone(closest_point_along_ray(pts, origin, Rotation2d(math.pi), 0.05))


class SubsystemTests(unittest.TestCase):
    def setUp(self):
        reset()
        self.world = World()
        self.lidar = LidarSubsystem()

    def test_fresh_data(self):
        self.world.publish(front=0.12, wall=(0.13, 2.0), points=[Translation2d(0.55, 0.0)])
        self.lidar.periodic()
        self.assertTrue(self.lidar.isConnected())
        self.assertAlmostEqual(self.lidar.getFrontDistance(), 0.12)
        wall = self.lidar.getWall()
        self.assertAlmostEqual(wall.distance, 0.13)
        self.assertAlmostEqual(wall.angle.degrees(), 2.0)
        self.assertEqual(len(self.lidar.getPoints()), 1)
        self.assertAlmostEqual(self.lidar.getPointsTimestamp(), 100.0 - 0.030)

    def test_goes_stale_when_heartbeat_stops(self):
        self.world.publish(front=0.12)
        self.lidar.periodic()
        wpilib.Timer.now += 0.5  # Pi script stopped: heartbeat no longer changes
        self.lidar.periodic()
        self.assertFalse(self.lidar.isConnected())
        self.assertIsNone(self.lidar.getFrontDistance())
        self.assertIsNone(self.lidar.getWall())
        self.assertEqual(self.lidar.getPoints(), [])

    def test_nothing_ahead(self):
        self.world.publish(front=None)
        self.lidar.periodic()
        self.assertIsNone(self.lidar.getFrontDistance())

    def test_field_points_published(self):
        lidar = LidarSubsystem(lambda: Pose2d(Translation2d(5, 5), Rotation2d(0)))
        self.world.publish(points=[Translation2d(0.5, 0.0)])
        lidar.periodic()
        (p,) = ntcore.PUBLISHED["/Lidar/FieldPoints"]
        self.assertAlmostEqual(p.x, 5.5)


class WallSimulation(unittest.TestCase):
    """Wall along field x = WALL_X. The robot starts somewhere behind it, turned."""

    WALL_X = 2.0

    def setUp(self):
        reset()
        self.world = World()
        self.lidar = LidarSubsystem()

    def wall_update(self, drive):
        def update():
            heading = math.degrees(drive.h)
            perp = self.WALL_X - drive.x
            if perp - LENGTH / 2 > 0.30 or abs(heading) > 50:
                self.world.publish()  # out of range
                return
            gap = perp - LENGTH / 2
            pts = [drive.to_robot(self.WALL_X, drive.y + dy) for dy in (-0.3, 0.0, 0.3)]
            self.world.publish(front=gap / math.cos(drive.h), wall=(gap, heading), points=pts)
        return update

    def test_square_to_wall(self):
        for start in (8.0, -8.0):
            drive = FakeDrive(x=self.WALL_X - LENGTH / 2 - 0.15, heading_deg=start)
            t = run(SquareToWall(self.lidar, object(), drive), self.lidar, drive, self.wall_update(drive))
            self.assertIsNotNone(t, "did not finish from %+.0f deg" % start)
            self.assertLess(abs(math.degrees(drive.h)), 1.0)

    def test_square_then_drive(self):
        drive = FakeDrive(x=self.WALL_X - LENGTH / 2 - 0.25, heading_deg=6.0)
        update = self.wall_update(drive)
        self.assertIsNotNone(run(SquareToWall(self.lidar, object(), drive), self.lidar, drive, update))
        self.assertIsNotNone(
            run(DriveToWallDistance(self.lidar, object(), drive, 0.05), self.lidar, drive, update))
        gap = self.WALL_X - drive.x - LENGTH / 2
        self.assertAlmostEqual(gap, 0.05, delta=0.012)

    def test_backs_up_when_too_close(self):
        drive = FakeDrive(x=self.WALL_X - LENGTH / 2 - 0.02)
        run(DriveToWallDistance(self.lidar, object(), drive, 0.10), self.lidar, drive, self.wall_update(drive))
        self.assertAlmostEqual(self.WALL_X - drive.x - LENGTH / 2, 0.10, delta=0.012)

    def test_stops_when_lidar_lost(self):
        drive = FakeDrive(x=0.0)
        cmd = DriveToWallDistance(self.lidar, object(), drive, 0.05)
        t = run(cmd, self.lidar, drive, lambda: None, max_seconds=2.0)  # nothing ever published
        self.assertIsNotNone(t)
        self.assertEqual(drive.last.vx, 0)


class FakeTarget:
    def __init__(self, yaw):
        self.yaw = yaw

    def getYaw(self):
        return self.yaw


class FakeResult:
    def __init__(self, yaw=None):
        self.yaw = yaw

    def hasTargets(self):
        return self.yaw is not None

    def getBestTarget(self):
        return FakeTarget(self.yaw)


class FakeCamera:
    """Sees the piece within +/- 35 deg, but loses it inside 0.35 m (like a real camera)."""

    def __init__(self, drive, robot_to_camera, piece):
        self.drive, self.r2c, self.piece = drive, robot_to_camera, piece

    def getAllUnreadResults(self):
        p = self.drive.to_robot(*self.piece) - self.r2c.translation()
        bearing = p.angle().radians() - self.r2c.rotation().radians()
        if p.norm() < 0.35 or abs(math.degrees(bearing)) > 35:
            return [FakeResult()]
        return [FakeResult(-math.degrees(bearing))]  # PhotonVision yaw is positive to the right


class IntakeSimulation(unittest.TestCase):
    def setUp(self):
        reset()
        self.world = World()
        self.lidar = LidarSubsystem()

    def run_intake(self, piece, start_heading_deg=0.0, stop=0.10):
        drive = FakeDrive(heading_deg=start_heading_deg)
        r2c = Transform2d(Translation2d(0.30, 0.0), Rotation2d(0))
        camera = FakeCamera(drive, r2c, piece)

        def update():
            # The lidar sees the piece as a few points on its near face, within 0.30 m of the bumper
            p = drive.to_robot(*piece)
            if p.x - LENGTH / 2 <= 0.30 and p.x > LENGTH / 2:
                pts = [Translation2d(p.x - 0.05, p.y + dy) for dy in (-0.04, 0.0, 0.04)]
                self.world.publish(points=pts)
            else:
                self.world.publish()

        cmd = LidarIntakeAssist(self.lidar, camera, r2c, object(), drive, stop)
        t = run(cmd, self.lidar, drive, update)
        return t, drive

    def test_drives_to_piece_and_stops(self):
        piece = (2.0, 0.6)  # ahead and to the left
        t, drive = self.run_intake(piece)
        self.assertIsNotNone(t, "never finished")
        p = drive.to_robot(*piece)
        face = Translation2d(p.x - 0.05, p.y)
        gap = (face - Translation2d(0.30, 0.0)).norm()
        self.assertAlmostEqual(gap, 0.10, delta=0.03)
        self.assertLess(abs(p.y), 0.06, "piece should end up centered in front of the robot")

    def test_turns_toward_piece_on_the_right(self):
        drive = FakeDrive()
        r2c = Transform2d(Translation2d(0.30, 0.0), Rotation2d(0))
        camera = FakeCamera(drive, r2c, (2.0, -0.8))
        cmd = LidarIntakeAssist(self.lidar, camera, r2c, object(), drive, 0.10)
        self.world.publish()
        self.lidar.periodic()
        cmd.initialize()
        cmd.execute()
        self.assertLess(drive.last.omega, 0, "piece on the right means turn clockwise")

    def test_waits_if_never_seen(self):
        drive = FakeDrive()
        r2c = Transform2d(Translation2d(0.30, 0.0), Rotation2d(0))
        camera = FakeCamera(drive, r2c, (-3.0, 0.0))  # behind the robot
        cmd = LidarIntakeAssist(self.lidar, camera, r2c, object(), drive, 0.10)
        self.world.publish()
        self.lidar.periodic()
        cmd.initialize()
        cmd.execute()
        self.assertEqual((drive.last.vx, drive.last.omega), (0, 0))
        self.assertFalse(cmd.isFinished())


if __name__ == "__main__":
    unittest.main()

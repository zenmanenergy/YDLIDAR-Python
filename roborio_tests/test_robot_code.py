#!/usr/bin/env python3
"""
Logic tests for the example robots in ../roborio/examples. Run from the repo root:

    python3 -m unittest discover roborio_tests -v

These run anywhere, without RobotPy: roborio_tests/fakes holds small stand-ins for
wpilib, ntcore, wpimath and photonlibpy that use the same names as RobotPy 2026.
Each test loads an example's robot.py, replaces its drive() with a simulated
robot, and runs it like the real robot loop (every 20 ms) until it finishes.
They check the logic (signs, handoffs, stopping), not RobotPy itself, so still
test on the real robot.
"""

import importlib.util
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
EXAMPLES = os.path.join(os.path.dirname(HERE), "roborio", "examples")
sys.path.insert(0, os.path.join(HERE, "fakes"))

import ntcore  # noqa: E402  (the stand-in)
import wpilib  # noqa: E402

LENGTH = 0.84  # robot length including bumpers, as configured on the Pi
DT = 0.02


def load_example(name):
    """Import roborio/examples/<name>/robot.py as its own module."""
    path = os.path.join(EXAMPLES, name, "robot.py")
    spec = importlib.util.spec_from_file_location("example_" + name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def reset():
    ntcore.STORE.clear()
    ntcore.TIMES.clear()
    wpilib.SmartDashboard.values.clear()
    wpilib.Timer.now = 100.0


class SimRobot:
    """A robot pose on the field that moves however drive() tells it to."""

    def __init__(self, x=0.0, y=0.0, heading_deg=0.0):
        self.x, self.y, self.h = x, y, math.radians(heading_deg)
        self.speeds = (0.0, 0.0, 0.0)

    def drive(self, forward, left, turn):
        self.speeds = (forward, left, turn)

    def step(self):
        f, l, t = self.speeds
        self.h += t * DT
        self.x += (f * math.cos(self.h) - l * math.sin(self.h)) * DT
        self.y += (f * math.sin(self.h) + l * math.cos(self.h)) * DT

    def to_robot(self, fx, fy):
        """Field point -> robot-relative (x, y)."""
        dx, dy = fx - self.x, fy - self.y
        c, s = math.cos(-self.h), math.sin(-self.h)
        return dx * c - dy * s, dx * s + dy * c


class Pi:
    """Plays the role of lidar.py on the Orange Pi."""

    def __init__(self):
        self.beat = 0

    def publish(self, front=None, wall=None, points=()):
        self.beat += 1
        ntcore.STORE["/lidar/heartbeat"] = self.beat
        ntcore.STORE["/lidar/connected"] = True
        ntcore.STORE["/lidar/frontDistance"] = -1.0 if front is None else front
        ntcore.STORE["/lidar/wall/valid"] = wall is not None
        if wall is not None:
            ntcore.STORE["/lidar/wall/distance"] = wall[0]
            ntcore.STORE["/lidar/wall/angleDeg"] = wall[1]
        flat = []
        for x, y in points:
            flat += [x, y]
        ntcore.STORE["/lidar/merged/points"] = flat


def make_robot(example, sim):
    robot = load_example(example).MyRobot()
    robot.robotInit()
    robot.drive = sim.drive
    robot.controller.a = True  # hold A
    return robot


def run(robot, sim, update, seconds=10.0, done=None):
    """Robot loop: Pi publishes, robot runs, physics moves. Returns time when done() is true."""
    t = 0.0
    while t < seconds:
        update()
        if hasattr(robot, "robotPeriodic"):
            robot.robotPeriodic()
        robot.teleopPeriodic()
        sim.step()
        wpilib.Timer.now += DT
        t += DT
        if done is not None and done():
            return t
    return None


def stopped(sim):
    return lambda: sim.speeds == (0, 0, 0)


class WallWorld:
    """A wall along field x = WALL_X, seen by the lidar when close enough."""

    WALL_X = 2.0

    def __init__(self, pi, sim):
        self.pi, self.sim = pi, sim

    def gap(self):
        return self.WALL_X - self.sim.x - LENGTH / 2

    def update(self):
        heading = math.degrees(self.sim.h)
        if self.gap() > 0.30 or abs(heading) > 50:
            self.pi.publish()  # out of range: nothing seen
            return
        pts = [self.sim.to_robot(self.WALL_X, self.sim.y + dy) for dy in (-0.3, 0.0, 0.3)]
        self.pi.publish(front=self.gap() / math.cos(self.sim.h), wall=(self.gap(), heading), points=pts)


class ReadLidarTests(unittest.TestCase):
    def setUp(self):
        reset()

    def test_shows_values(self):
        robot = load_example("read_lidar").MyRobot()
        robot.robotInit()
        Pi().publish(front=0.12, wall=(0.13, 2.0), points=[(0.5, 0.0), (0.5, 0.1)])
        robot.robotPeriodic()
        v = wpilib.SmartDashboard.values
        self.assertTrue(v["Lidar/Connected"])
        self.assertEqual(v["Lidar/FrontDistance"], 0.12)
        self.assertTrue(v["Lidar/WallFound"])
        self.assertEqual(v["Lidar/WallAngleDeg"], 2.0)
        self.assertEqual(v["Lidar/PointCount"], 2)

    def test_stale_shows_minus_one(self):
        robot = load_example("read_lidar").MyRobot()
        robot.robotInit()
        Pi().publish(front=0.12)
        robot.robotPeriodic()
        wpilib.Timer.now += 0.5  # Pi stopped: heartbeat stops changing
        robot.robotPeriodic()
        v = wpilib.SmartDashboard.values
        self.assertFalse(v["Lidar/Connected"])
        self.assertEqual(v["Lidar/FrontDistance"], -1)


class DriveToWallDistanceTests(unittest.TestCase):
    def setUp(self):
        reset()

    def test_drives_forward_to_gap(self):
        sim = SimRobot(x=WallWorld.WALL_X - LENGTH / 2 - 0.25)
        world = WallWorld(Pi(), sim)
        robot = make_robot("drive_to_wall_distance", sim)
        self.assertIsNotNone(run(robot, sim, world.update, done=stopped(sim)))
        self.assertAlmostEqual(world.gap(), 0.05, delta=0.012)

    def test_backs_up_when_too_close(self):
        sim = SimRobot(x=WallWorld.WALL_X - LENGTH / 2 - 0.01)
        world = WallWorld(Pi(), sim)
        robot = make_robot("drive_to_wall_distance", sim)
        run(robot, sim, world.update, done=stopped(sim))
        self.assertAlmostEqual(world.gap(), 0.05, delta=0.012)

    def test_stops_when_button_released(self):
        sim = SimRobot(x=WallWorld.WALL_X - LENGTH / 2 - 0.25)
        world = WallWorld(Pi(), sim)
        robot = make_robot("drive_to_wall_distance", sim)
        run(robot, sim, world.update, seconds=0.1)
        self.assertGreater(sim.speeds[0], 0)
        robot.controller.a = False
        run(robot, sim, world.update, seconds=0.02)
        self.assertEqual(sim.speeds, (0, 0, 0))

    def test_stops_when_lidar_missing(self):
        sim = SimRobot()
        robot = make_robot("drive_to_wall_distance", sim)
        run(robot, sim, lambda: None, seconds=0.5)  # Pi never publishes
        self.assertEqual(sim.speeds, (0, 0, 0))
        self.assertEqual(sim.x, 0.0)


class SquareToWallTests(unittest.TestCase):
    def setUp(self):
        reset()

    def test_squares_up_from_both_sides(self):
        for start in (8.0, -8.0):
            sim = SimRobot(x=WallWorld.WALL_X - LENGTH / 2 - 0.15, heading_deg=start)
            robot = make_robot("square_to_wall", sim)
            self.assertIsNotNone(run(robot, sim, WallWorld(Pi(), sim).update, done=stopped(sim)),
                                 "never finished from %+.0f deg" % start)
            self.assertLess(abs(math.degrees(sim.h)), 1.0)

    def test_turns_the_right_way(self):
        sim = SimRobot(x=WallWorld.WALL_X - LENGTH / 2 - 0.15, heading_deg=10)
        robot = make_robot("square_to_wall", sim)
        run(robot, sim, WallWorld(Pi(), sim).update, seconds=0.02)
        self.assertLess(sim.speeds[2], 0, "turned left of square means turn right (negative)")


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

    def __init__(self, sim, cam_x, cam_y, piece):
        self.sim, self.cam_x, self.cam_y, self.piece = sim, cam_x, cam_y, piece
        self.sightings = []

    def getAllUnreadResults(self):
        px, py = self.sim.to_robot(*self.piece)
        dx, dy = px - self.cam_x, py - self.cam_y
        bearing = math.atan2(dy, dx)
        seen = math.hypot(dx, dy) >= 0.35 and abs(math.degrees(bearing)) <= 35
        self.sightings.append(seen)
        if not seen:
            return [FakeResult()]
        return [FakeResult(-math.degrees(bearing))]  # PhotonVision yaw: positive to the right


class IntakeAssistTests(unittest.TestCase):
    def setUp(self):
        reset()
        self.module = load_example("lidar_intake_assist")

    def setup_intake(self, piece):
        sim = SimRobot()
        robot = self.module.MyRobot()
        robot.robotInit()
        robot.drive = sim.drive
        robot.controller.a = True
        camera = FakeCamera(sim, self.module.CAMERA_X, self.module.CAMERA_Y, piece)
        robot.camera = camera
        pi = Pi()

        def update():
            # The lidar sees the near face of the piece, within 0.30 m of the bumper
            px, py = sim.to_robot(*piece)
            if LENGTH / 2 < px <= LENGTH / 2 + 0.30:
                pi.publish(points=[(px - 0.05, py + dy) for dy in (-0.04, 0.0, 0.04)])
            else:
                pi.publish()

        return sim, robot, camera, update

    def test_drives_to_piece_and_stops(self):
        piece = (2.0, 0.6)
        sim, robot, camera, update = self.setup_intake(piece)
        t = run(robot, sim, update, done=lambda: robot.arrived)
        self.assertIsNotNone(t, "never arrived")
        px, py = sim.to_robot(*piece)
        gap = math.hypot(px - 0.05 - self.module.CAMERA_X, py - self.module.CAMERA_Y)
        self.assertAlmostEqual(gap, self.module.STOP_DISTANCE, delta=0.03)
        self.assertLess(abs(py), 0.06, "piece should end up centered in front of the robot")
        # The camera lost the piece before the end, so the lidar finished the approach
        self.assertFalse(camera.sightings[-1])
        self.assertTrue(any(camera.sightings))

    def test_stays_stopped_after_arriving(self):
        sim, robot, camera, update = self.setup_intake((2.0, 0.0))
        self.assertIsNotNone(run(robot, sim, update, done=lambda: robot.arrived))
        x_at_arrival = sim.x
        run(robot, sim, update, seconds=0.5)  # keep holding A
        self.assertEqual(sim.speeds, (0, 0, 0))
        self.assertEqual(sim.x, x_at_arrival)

    def test_new_press_starts_over(self):
        sim, robot, camera, update = self.setup_intake((2.0, 0.0))
        self.assertIsNotNone(run(robot, sim, update, done=lambda: robot.arrived))
        robot.controller.a = False  # release A
        run(robot, sim, update, seconds=0.02)
        robot.controller.a = True  # press again
        update()
        robot.teleopPeriodic()
        # It forgot the old piece. (Up close the camera can't see it, so it waits.)
        self.assertFalse(robot.arrived)
        self.assertIsNone(robot.piece_direction)

    def test_turns_toward_piece_on_the_right(self):
        sim, robot, camera, update = self.setup_intake((2.0, -0.8))
        run(robot, sim, update, seconds=0.02)
        self.assertLess(sim.speeds[2], 0, "piece on the right means turn clockwise")

    def test_waits_if_never_seen(self):
        sim, robot, camera, update = self.setup_intake((-3.0, 0.0))  # behind the robot
        run(robot, sim, update, seconds=0.5)
        self.assertEqual(sim.speeds, (0, 0, 0))


if __name__ == "__main__":
    unittest.main()

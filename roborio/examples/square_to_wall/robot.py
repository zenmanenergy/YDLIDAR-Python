#!/usr/bin/env python3
"""
EXAMPLE: square_to_wall

Hold the A button: the robot turns in place until it faces the wall in front
of it squarely. Let go of A: the robot stops.

How it works: the Orange Pi fits a straight line through the lidar points in
front of the robot. If the robot is turned, that line is tilted, and the Pi
sends us the tilt as "wall/angleDeg". We turn the opposite way, faster when the
tilt is bigger, until the tilt is nearly zero.

    Robot turned 10 degrees left of square  ->  angle = +10  ->  turn right
    Robot turned 10 degrees right of square ->  angle = -10  ->  turn left

Before using on a real robot, replace the drive() method at the bottom with a
call to your own drivetrain. Until then, it only shows the speeds it would use
on the SmartDashboard, so this example is safe to run on a robot on blocks.

Units: degrees for the wall angle, radians per second for turning.
"""

import math

import wpilib
import ntcore

KP = 3.0  # radians per second of turning for each radian of error
MAX_TURN_SPEED = 1.5  # radians per second (about 86 degrees per second)
TOLERANCE_DEGREES = 1.0  # "square enough" to stop

STALE_SECONDS = 0.25  # if the Orange Pi goes quiet this long, the data is missing


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.XboxController(0)

        # Lidar values published by the Orange Pi
        table = ntcore.NetworkTableInstance.getDefault().getTable("lidar")
        self.heartbeat = table.getIntegerTopic("heartbeat").subscribe(0)
        self.connected = table.getBooleanTopic("connected").subscribe(False)
        self.wall_valid = table.getBooleanTopic("wall/valid").subscribe(False)
        self.wall_angle = table.getDoubleTopic("wall/angleDeg").subscribe(0.0)

        self.last_heartbeat = None
        self.last_heartbeat_time = -1000.0

    def lidarIsFresh(self):
        """True if the Orange Pi is still sending data (its heartbeat keeps changing)."""
        now = wpilib.Timer.getFPGATimestamp()
        beat = self.heartbeat.get()
        if beat != self.last_heartbeat:
            self.last_heartbeat = beat
            self.last_heartbeat_time = now
        return (now - self.last_heartbeat_time) < STALE_SECONDS and self.connected.get()

    def getWallAngle(self):
        """
        How far the robot is turned away from square with the wall, in degrees.
        Positive = turned counterclockwise (left). None if no flat wall is visible.
        """
        if not self.lidarIsFresh() or not self.wall_valid.get():
            return None
        return self.wall_angle.get()

    def teleopPeriodic(self):
        # teleopPeriodic runs every 20 ms while the robot is enabled in teleop

        if not self.controller.getAButton():
            self.drive(0, 0, 0)  # A not held: stop
            return

        angle = self.getWallAngle()
        if angle is None:
            self.drive(0, 0, 0)  # no wall in view: stop rather than guess
            return

        wpilib.SmartDashboard.putNumber("Example/WallAngleDeg", angle)

        if abs(angle) < TOLERANCE_DEGREES:
            self.drive(0, 0, 0)  # square
            return

        # Turn the OPPOSITE way to the angle (that's the minus sign)
        turn = -KP * math.radians(angle)
        turn = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn))
        self.drive(0, 0, turn)

    def drive(self, forward, left, turn):
        """
        REPLACE THIS with a call to your drivetrain.

        forward: meters per second (+ = forward)
        left:    meters per second (+ = left; only swerve/mecanum can do this)
        turn:    radians per second (+ = counterclockwise)

        For example, with a swerve drivetrain:
            self.drivetrain.driveRobotRelative(ChassisSpeeds(forward, left, turn))
        """
        wpilib.SmartDashboard.putNumber("Drive/Forward", forward)
        wpilib.SmartDashboard.putNumber("Drive/Left", left)
        wpilib.SmartDashboard.putNumber("Drive/Turn", turn)

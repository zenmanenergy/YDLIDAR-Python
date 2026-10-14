#!/usr/bin/env python3
"""
EXAMPLE: drive_to_wall_distance

Hold the A button: the robot drives straight forward (or backward) until its
front bumper is exactly TARGET_GAP away from the wall or object in front of it.
Let go of A: the robot stops.

How it works: every 20 ms we read the distance from the lidar, compare it with
the distance we want, and drive faster the bigger the difference is. This is
called a "proportional controller": speed = KP x error.

Before using on a real robot, replace the drive() method at the bottom with a
call to your own drivetrain. Until then, it only shows the speeds it would use
on the SmartDashboard, so this example is safe to run on a robot on blocks.

Units: meters and meters per second.
"""

import wpilib
import ntcore

TARGET_GAP = 0.05  # meters: how close we want the front bumper to the wall
KP = 2.0  # how hard to push: meters per second of speed for each meter of error
MAX_SPEED = 0.5  # meters per second. Keep it slow this close to things!
TOLERANCE = 0.01  # meters: "close enough" to stop

STALE_SECONDS = 0.25  # if the Orange Pi goes quiet this long, the data is missing


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.XboxController(0)

        # Lidar values published by the Orange Pi
        table = ntcore.NetworkTableInstance.getDefault().getTable("lidar")
        self.heartbeat = table.getIntegerTopic("heartbeat").subscribe(0)
        self.connected = table.getBooleanTopic("connected").subscribe(False)
        self.front_distance = table.getDoubleTopic("frontDistance").subscribe(-1.0)
        self.wall_valid = table.getBooleanTopic("wall/valid").subscribe(False)
        self.wall_distance = table.getDoubleTopic("wall/distance").subscribe(0.0)

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

    def getDistanceAhead(self):
        """
        Distance from the front bumper to whatever is ahead, in meters.
        Returns None if the lidar can't see anything or isn't working.
        """
        if not self.lidarIsFresh():
            return None

        # Best: a flat wall. The Pi fits a line through many points, which is
        # steadier than any single point.
        if self.wall_valid.get():
            return self.wall_distance.get()

        # Next best: the single closest point straight ahead. -1 means nothing.
        d = self.front_distance.get()
        if d < 0:
            return None
        return d

    def teleopPeriodic(self):
        # teleopPeriodic runs every 20 ms while the robot is enabled in teleop

        if not self.controller.getAButton():
            self.drive(0, 0, 0)  # A not held: stop
            return

        distance = self.getDistanceAhead()
        if distance is None:
            self.drive(0, 0, 0)  # can't see the wall: stop rather than guess
            return

        # error is positive when we're too far away, negative when too close
        error = distance - TARGET_GAP
        wpilib.SmartDashboard.putNumber("Example/Error", error)

        if abs(error) < TOLERANCE:
            self.drive(0, 0, 0)  # we're there
            return

        speed = KP * error
        # Never go faster than MAX_SPEED in either direction
        speed = max(-MAX_SPEED, min(MAX_SPEED, speed))
        self.drive(speed, 0, 0)

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

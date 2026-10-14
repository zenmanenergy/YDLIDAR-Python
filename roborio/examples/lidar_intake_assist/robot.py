#!/usr/bin/env python3
"""
EXAMPLE: lidar_intake_assist

Hold the A button: the robot drives to a game piece and stops with it right in
front of the intake. Let go of A: the robot stops.

It uses two sensors, each for what it's good at:

  Camera (PhotonVision)  sees the piece from far away and tells us which
                         DIRECTION it is, but can't measure distance well up close.
  Lidar                  only sees about 300 mm ahead, but measures DISTANCE
                         to within a few millimeters.

So: far away, we steer toward the piece using the camera. Once the lidar sees
something in that same direction, we switch to the lidar for the final approach
and stop at exactly STOP_DISTANCE. Cameras often lose sight of a piece right at
the bumper, so the lidar can finish the job by itself.

Before using on a real robot:
  1. Set CAMERA_NAME, CAMERA_X, CAMERA_Y, CAMERA_YAW_DEG for your camera.
  2. Mount the lidar low enough to actually hit the game piece.
  3. Replace the drive() method at the bottom with a call to your drivetrain.
Until then, drive() only shows the speeds on the SmartDashboard, so this
example is safe to run on a robot on blocks.

Coordinates (WPILib convention): the robot's center is (0, 0). +x is forward,
+y is left, and counterclockwise angles are positive. Units: meters, radians.
"""

import math

import wpilib
import ntcore
from photonlibpy import PhotonCamera

# --- Camera setup: CHANGE THESE to match your robot ---
CAMERA_NAME = "intake"  # the camera's name in the PhotonVision dashboard
CAMERA_X = 0.30  # meters forward of the robot's center
CAMERA_Y = 0.0  # meters left of the robot's center
CAMERA_YAW_DEG = 0.0  # which way it points: 0 = straight ahead, + = turned left

# --- Driving ---
STOP_DISTANCE = 0.10  # meters from the camera to the piece when we stop. Tune so
#                       the piece ends up in the intake.
CRUISE_SPEED = 1.0  # m/s while only the camera sees the piece
APPROACH_KP = 3.0  # m/s per meter of distance left, once the lidar sees it
MAX_APPROACH_SPEED = 0.6  # m/s
TURN_KP = 3.0  # rad/s of turning per radian the piece is off-center
MAX_TURN_SPEED = 2.0  # rad/s
STOP_TOLERANCE = 0.015  # meters: "close enough"

# --- Matching camera and lidar ---
LINE_HALF_WIDTH = 0.08  # meters: a lidar point counts as "the piece" if it is
#                         within this distance of the camera's line of sight
CAMERA_TIMEOUT = 0.5  # seconds: how long to trust the last camera sighting

STALE_SECONDS = 0.25  # if the Orange Pi goes quiet this long, the data is missing


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.controller = wpilib.XboxController(0)
        self.camera = PhotonCamera(CAMERA_NAME)

        # Lidar values published by the Orange Pi
        table = ntcore.NetworkTableInstance.getDefault().getTable("lidar")
        self.heartbeat = table.getIntegerTopic("heartbeat").subscribe(0)
        self.connected = table.getBooleanTopic("connected").subscribe(False)
        # All the points the lidars see, as one flat list: [x0, y0, x1, y1, ...]
        self.points = table.getDoubleArrayTopic("merged/points").subscribe([])

        self.last_heartbeat = None
        self.last_heartbeat_time = -1000.0
        self.reset()

    def reset(self):
        """Forget everything about the last piece. Called each time A is pressed."""
        self.piece_direction = None  # angle from the camera to the piece, or None
        self.last_seen_time = -1000.0  # when the camera last saw the piece
        self.arrived = False

    def lidarIsFresh(self):
        """True if the Orange Pi is still sending data (its heartbeat keeps changing)."""
        now = wpilib.Timer.getFPGATimestamp()
        beat = self.heartbeat.get()
        if beat != self.last_heartbeat:
            self.last_heartbeat = beat
            self.last_heartbeat_time = now
        return (now - self.last_heartbeat_time) < STALE_SECONDS and self.connected.get()

    def getLidarPoints(self):
        """All lidar points as a list of (x, y) pairs, robot-relative, meters."""
        if not self.lidarIsFresh():
            return []
        flat = self.points.get()
        pairs = []
        for i in range(0, len(flat) - 1, 2):
            pairs.append((flat[i], flat[i + 1]))
        return pairs

    def findPointAlongLine(self, start_x, start_y, direction):
        """
        Look along a line starting at (start_x, start_y) and pointing in
        `direction` (radians). Return the closest lidar point that is within
        LINE_HALF_WIDTH of that line, as (x, y), or None if there isn't one.
        """
        cos_d = math.cos(direction)
        sin_d = math.sin(direction)
        best_point = None
        best_along = math.inf
        for x, y in self.getLidarPoints():
            dx = x - start_x
            dy = y - start_y
            # How far along the line the point is, and how far off to the side
            along = dx * cos_d + dy * sin_d
            sideways = -dx * sin_d + dy * cos_d
            if along > 0 and abs(sideways) <= LINE_HALF_WIDTH and along < best_along:
                best_along = along
                best_point = (x, y)
        return best_point

    def teleopPeriodic(self):
        # teleopPeriodic runs every 20 ms while the robot is enabled in teleop

        if self.controller.getAButtonPressed():
            self.reset()  # new press: start fresh
        if not self.controller.getAButton() or self.arrived:
            self.drive(0, 0, 0)
            return

        now = wpilib.Timer.getFPGATimestamp()

        # 1. CAMERA: which direction is the piece?
        results = self.camera.getAllUnreadResults()
        if results:
            latest = results[-1]
            if latest.hasTargets():
                # PhotonVision measures yaw as positive to the RIGHT, but WPILib
                # angles are positive to the LEFT, so we flip the sign.
                yaw = -latest.getBestTarget().getYaw()
                self.piece_direction = math.radians(CAMERA_YAW_DEG + yaw)
                self.last_seen_time = now

        if self.piece_direction is None:
            self.drive(0, 0, 0)  # haven't seen a piece yet
            return

        # 2. LIDAR: is there something along the camera's line of sight?
        piece = self.findPointAlongLine(CAMERA_X, CAMERA_Y, self.piece_direction)

        camera_lost_it = (now - self.last_seen_time) > CAMERA_TIMEOUT
        if camera_lost_it and piece is None:
            self.drive(0, 0, 0)  # neither sensor has the piece: stop
            return

        if piece is not None:
            piece_x, piece_y = piece
            # Exact distance from the camera to the piece
            distance = math.hypot(piece_x - CAMERA_X, piece_y - CAMERA_Y)
            # Keep looking along the line to where the piece really is, so we
            # keep tracking it even after the camera loses sight of it
            self.piece_direction = math.atan2(piece_y - CAMERA_Y, piece_x - CAMERA_X)
            # Angle from the robot's CENTER to the piece: 0 means dead ahead
            off_center = math.atan2(piece_y, piece_x)
        else:
            distance = None
            off_center = self.piece_direction  # camera only: close enough far away

        wpilib.SmartDashboard.putBoolean("Example/LidarHasPiece", piece is not None)

        # 3. STEER so the piece ends up straight ahead
        turn = TURN_KP * off_center
        turn = max(-MAX_TURN_SPEED, min(MAX_TURN_SPEED, turn))

        # 4. DRIVE: cruise on the camera, then slow down on the lidar and stop
        if distance is None:
            forward = CRUISE_SPEED
        else:
            distance_left = distance - STOP_DISTANCE
            if distance_left <= STOP_TOLERANCE:
                self.arrived = True
                self.drive(0, 0, 0)
                return
            forward = min(MAX_APPROACH_SPEED, APPROACH_KP * distance_left)

        # Slow down while still turning hard, so we don't swing past the piece
        forward = forward * max(0.0, math.cos(off_center))

        self.drive(forward, 0, turn)

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

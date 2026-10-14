import math
import typing

import commands2
import wpilib
from wpimath.geometry import Rotation2d, Transform2d
from wpimath.kinematics import ChassisSpeeds

from subsystems.lidar_subsystem import LidarSubsystem

CRUISE_SPEED = 1.0  # m/s while only the camera sees the piece
APPROACH_KP = 3.0  # (m/s) per meter, once the lidar has it
MAX_APPROACH_SPEED = 0.6  # m/s
TURN_KP = 3.0  # (rad/s) per radian
MAX_OMEGA = 2.0  # rad/s
CORRIDOR_HALF_WIDTH = 0.08  # meters either side of the camera ray
TARGET_TIMEOUT = 0.5  # seconds without a camera sighting
STOP_TOLERANCE = 0.015  # meters


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class LidarIntakeAssist(commands2.Command):
    """
    Drives to a game piece: PhotonVision steers from far away, the lidar takes over up close.

    Far away, the camera's object detection gives the direction to the piece and the robot drives
    toward it at cruise speed. Once the lidar sees something along that same direction (inside its
    ~300 mm range), the lidar's distance controls the final approach and the command stops at an
    exact gap. Cameras often lose the piece right before it reaches the bumper, so the lidar can
    finish the approach on its own.

    The lidar only sees objects at the height it is mounted. Mount it low enough to hit the game
    piece, or this command will only ever use the camera.
    """

    def __init__(
        self,
        lidar: LidarSubsystem,
        camera,
        robot_to_camera: Transform2d,
        drive_subsystem: commands2.Subsystem,
        robot_relative_drive: typing.Callable[[ChassisSpeeds], None],
        stop_distance_meters: float,
    ):
        """
        :param lidar: the lidar subsystem
        :param camera: a photonlibpy PhotonCamera running an object detection (or color) pipeline
        :param robot_to_camera: where the camera is on the robot (x, y, and the direction it faces)
        :param drive_subsystem: your drivetrain
        :param robot_relative_drive: drives the robot with robot-relative speeds
        :param stop_distance_meters: gap between the camera position and the piece at which to
            stop. Tune this so the piece ends up in the intake.
        """
        super().__init__()
        self.lidar = lidar
        self.camera = camera
        self.robot_to_camera = robot_to_camera
        self.drive = robot_relative_drive
        self.stop_distance = stop_distance_meters
        self.addRequirements(drive_subsystem)
        self._reset()

    def _reset(self):
        self.ray_direction = None  # robot-relative direction to the piece, from the camera
        self.last_seen = -math.inf
        self.lidar_distance = None

    def initialize(self):
        self._reset()

    def execute(self):
        now = wpilib.Timer.getFPGATimestamp()

        # 1. Camera: which way is the piece? PhotonVision yaw is positive to the RIGHT,
        #    WPILib angles are positive to the LEFT, so negate it.
        results = self.camera.getAllUnreadResults()
        if results:
            latest = results[-1]
            if latest.hasTargets():
                yaw_deg = latest.getBestTarget().getYaw()
                self.ray_direction = self.robot_to_camera.rotation() + Rotation2d.fromDegrees(-yaw_deg)
                self.last_seen = now

        if self.ray_direction is None:
            self.drive(ChassisSpeeds())  # never seen the piece yet
            return

        # 2. Lidar: is there something along that same line, close enough to measure precisely?
        camera_pos = self.robot_to_camera.translation()
        piece = self.lidar.getClosestPointAlongRay(camera_pos, self.ray_direction, CORRIDOR_HALF_WIDTH)
        self.lidar_distance = None if piece is None else (piece - camera_pos).norm()

        # Stop if neither sensor has the piece
        if now - self.last_seen > TARGET_TIMEOUT and piece is None:
            self.drive(ChassisSpeeds())
            return

        # Steer so the piece is straight ahead of the robot center. When the lidar has it, use the
        # measured point (exact) and re-aim the ray at it, so it stays tracked as the robot turns.
        # Otherwise use the camera direction (good enough far away).
        bearing = self.ray_direction
        if piece is not None:
            self.ray_direction = (piece - camera_pos).angle()
            bearing = piece.angle()
        omega = clamp(TURN_KP * bearing.radians(), -MAX_OMEGA, MAX_OMEGA)

        # 3. Forward speed: cruise on camera, then hand off to the lidar for the final approach.
        if self.lidar_distance is not None:
            error = self.lidar_distance - self.stop_distance
            vx = clamp(APPROACH_KP * error, 0.0, MAX_APPROACH_SPEED)
        else:
            vx = CRUISE_SPEED
        # Slow down while still turning hard, so we don't sweep past the piece
        vx *= max(0.0, bearing.cos())

        self.drive(ChassisSpeeds(vx, 0, omega))

    def isFinished(self) -> bool:
        arrived = (
            self.lidar_distance is not None
            and self.lidar_distance <= self.stop_distance + STOP_TOLERANCE
        )
        lost = (
            self.ray_direction is not None
            and wpilib.Timer.getFPGATimestamp() - self.last_seen > TARGET_TIMEOUT
            and self.lidar_distance is None
        )
        return arrived or lost

    def end(self, interrupted: bool):
        self.drive(ChassisSpeeds())

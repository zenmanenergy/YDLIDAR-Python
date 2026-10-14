import math
import typing

import commands2
from wpimath.kinematics import ChassisSpeeds

from subsystems.lidar_subsystem import LidarSubsystem

KP = 3.0  # (rad/s) per radian of error
MAX_OMEGA = 1.5  # rad/s
TOLERANCE_RAD = math.radians(1.0)


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class SquareToWall(commands2.Command):
    """
    Rotates in place until the robot faces the wall ahead squarely.

    The lidar fits a straight line to the points in front of the robot. The angle of that line
    tells us how far the robot is turned away from square, and we turn the opposite way.

    Ends when square within tolerance, or if no wall is visible.
    """

    def __init__(
        self,
        lidar: LidarSubsystem,
        drive_subsystem: commands2.Subsystem,
        robot_relative_drive: typing.Callable[[ChassisSpeeds], None],
    ):
        super().__init__()
        self.lidar = lidar
        self.drive = robot_relative_drive
        self.error = None
        self.addRequirements(drive_subsystem)

    def initialize(self):
        self.error = None

    def execute(self):
        wall = self.lidar.getWall()
        if wall is None:
            self.error = None
            self.drive(ChassisSpeeds())
            return
        # Positive angle means the robot is turned counterclockwise, so turn clockwise (negative)
        self.error = wall.angle.radians()
        self.drive(ChassisSpeeds(0, 0, clamp(-KP * self.error, -MAX_OMEGA, MAX_OMEGA)))

    def isFinished(self) -> bool:
        if self.lidar.getWall() is None:
            return True
        return self.error is not None and abs(self.error) < TOLERANCE_RAD

    def end(self, interrupted: bool):
        self.drive(ChassisSpeeds())

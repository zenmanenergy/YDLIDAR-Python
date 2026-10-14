import typing

import commands2
from wpimath.kinematics import ChassisSpeeds

from subsystems.lidar_subsystem import LidarSubsystem

KP = 2.0  # (m/s) per meter of error
MAX_SPEED = 0.5  # m/s, keep it gentle this close to things
TOLERANCE = 0.01  # meters


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


class DriveToWallDistance(commands2.Command):
    """
    Drives straight forward or backward until the front bumper is a set distance from whatever
    is ahead. Uses the fitted wall when there is one (less noisy), otherwise the closest point.

    Ends when the distance is within tolerance, or if lidar data is lost.
    """

    def __init__(
        self,
        lidar: LidarSubsystem,
        drive_subsystem: commands2.Subsystem,
        robot_relative_drive: typing.Callable[[ChassisSpeeds], None],
        target_meters: float,
    ):
        """
        :param lidar: the lidar subsystem
        :param drive_subsystem: your drivetrain, so this command owns it while running
        :param robot_relative_drive: drives the robot with robot-relative speeds
        :param target_meters: desired gap between the front bumper and the object ahead
        """
        super().__init__()
        self.lidar = lidar
        self.drive = robot_relative_drive
        self.target = target_meters
        self.error = None
        self.addRequirements(drive_subsystem)

    def initialize(self):
        self.error = None

    def execute(self):
        wall = self.lidar.getWall()
        distance = wall.distance if wall is not None else self.lidar.getFrontDistance()

        if distance is None:
            self.error = None
            self.drive(ChassisSpeeds())
            return
        self.error = distance - self.target
        self.drive(ChassisSpeeds(clamp(KP * self.error, -MAX_SPEED, MAX_SPEED), 0, 0))

    def isFinished(self) -> bool:
        if not self.lidar.isConnected():
            return True
        return self.error is not None and abs(self.error) < TOLERANCE

    def end(self, interrupted: bool):
        self.drive(ChassisSpeeds())

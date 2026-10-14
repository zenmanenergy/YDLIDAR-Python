"""
PLACEHOLDER drivetrain. Replace this with your real drivetrain subsystem.

It doesn't move any motors. It shows the commands' requested speeds on the
SmartDashboard and estimates a pose by adding those speeds up over time. That
lets you deploy this project as-is and bench-test the lidar: put an object in
front of the robot, press a button, and watch what speeds the commands ask for.

Your real drivetrain needs the same two methods:
    driveRobotRelative(speeds: ChassisSpeeds)   forward/left/turn, robot-relative
    getPose() -> Pose2d                         the robot's field position
"""

import math

import commands2
import wpilib
from wpimath.geometry import Pose2d, Rotation2d, Translation2d
from wpimath.kinematics import ChassisSpeeds

LOOP_SECONDS = 0.02


class ExampleDrivetrain(commands2.Subsystem):
    def __init__(self):
        super().__init__()
        self.speeds = ChassisSpeeds()
        self.x = 0.0
        self.y = 0.0
        self.heading = 0.0

    def driveRobotRelative(self, speeds: ChassisSpeeds):
        self.speeds = speeds

    def getPose(self) -> Pose2d:
        return Pose2d(Translation2d(self.x, self.y), Rotation2d(self.heading))

    def periodic(self):
        s = self.speeds
        wpilib.SmartDashboard.putNumber("Drive/RequestedVx", s.vx)
        wpilib.SmartDashboard.putNumber("Drive/RequestedVy", s.vy)
        wpilib.SmartDashboard.putNumber("Drive/RequestedOmega", s.omega)

        # Pretend the robot moved exactly as requested
        self.heading += s.omega * LOOP_SECONDS
        c, si = math.cos(self.heading), math.sin(self.heading)
        self.x += (s.vx * c - s.vy * si) * LOOP_SECONDS
        self.y += (s.vx * si + s.vy * c) * LOOP_SECONDS

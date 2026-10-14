"""
Creates the subsystems and binds commands to controller buttons.

Xbox controller on port 0:
    A (press)  square up to the wall ahead, then drive to 50 mm from it
    B (hold)   drive to a game piece: camera far away, lidar up close
"""

import commands2
import commands2.button
from photonlibpy import PhotonCamera
from wpimath.geometry import Rotation2d, Transform2d, Translation2d

from commands.drive_to_wall_distance import DriveToWallDistance
from commands.lidar_intake_assist import LidarIntakeAssist
from commands.square_to_wall import SquareToWall
from subsystems.example_drivetrain import ExampleDrivetrain
from subsystems.lidar_subsystem import LidarSubsystem

CONTROLLER_PORT = 0
INTAKE_CAMERA_NAME = "intake"  # must match the camera name in PhotonVision

# Where the intake camera is: 0.30 m forward of robot center, on the centerline, facing forward
ROBOT_TO_INTAKE_CAMERA = Transform2d(Translation2d(0.30, 0.0), Rotation2d(0))

WALL_GAP_METERS = 0.05  # A button: final gap between front bumper and wall
INTAKE_STOP_METERS = 0.10  # B button: gap between camera and game piece when stopped


class RobotContainer:
    def __init__(self):
        # Replace ExampleDrivetrain with your real drivetrain. It needs two things:
        #   driveRobotRelative(speeds: ChassisSpeeds) and getPose() -> Pose2d
        self.drivetrain = ExampleDrivetrain()

        # Passing getPose also publishes the lidar points in field coordinates for AdvantageScope
        self.lidar = LidarSubsystem(self.drivetrain.getPose)

        self.intake_camera = PhotonCamera(INTAKE_CAMERA_NAME)
        self.controller = commands2.button.CommandXboxController(CONTROLLER_PORT)

        self.configureButtonBindings()

    def configureButtonBindings(self):
        drive = self.drivetrain.driveRobotRelative

        self.controller.a().onTrue(
            SquareToWall(self.lidar, self.drivetrain, drive).andThen(
                DriveToWallDistance(self.lidar, self.drivetrain, drive, WALL_GAP_METERS)
            )
        )

        self.controller.b().whileTrue(
            LidarIntakeAssist(
                self.lidar,
                self.intake_camera,
                ROBOT_TO_INTAKE_CAMERA,
                self.drivetrain,
                drive,
                INTAKE_STOP_METERS,
            )
        )

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LidarSubsystem;
import java.util.OptionalDouble;
import java.util.function.Consumer;

/**
 * Drives straight forward or backward until the front bumper is a set distance from whatever is
 * ahead. Uses the fitted wall when there is one (less noisy), otherwise the closest point ahead.
 *
 * <p>Ends when the distance is within tolerance, or if lidar data is lost.
 */
public class DriveToWallDistance extends Command {
  private static final double KP = 2.0; // (m/s) per meter of error
  private static final double MAX_SPEED = 0.5; // m/s, keep it gentle this close to things
  private static final double TOLERANCE = 0.01; // meters

  private final LidarSubsystem lidar;
  private final Consumer<ChassisSpeeds> drive;
  private final double targetMeters;
  private double error = Double.NaN;

  /**
   * @param lidar the lidar subsystem
   * @param driveSubsystem your drivetrain, so this command owns it while running
   * @param robotRelativeDrive drives the robot with robot-relative speeds
   * @param targetMeters desired gap between the front bumper and the object ahead
   */
  public DriveToWallDistance(
      LidarSubsystem lidar,
      Subsystem driveSubsystem,
      Consumer<ChassisSpeeds> robotRelativeDrive,
      double targetMeters) {
    this.lidar = lidar;
    this.drive = robotRelativeDrive;
    this.targetMeters = targetMeters;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    error = Double.NaN;
  }

  @Override
  public void execute() {
    OptionalDouble distance =
        lidar
            .getWall()
            .map(w -> OptionalDouble.of(w.distanceMeters()))
            .orElse(lidar.getFrontDistance());

    if (distance.isEmpty()) {
      error = Double.NaN;
      drive.accept(new ChassisSpeeds());
      return;
    }
    error = distance.getAsDouble() - targetMeters;
    double vx = MathUtil.clamp(KP * error, -MAX_SPEED, MAX_SPEED);
    drive.accept(new ChassisSpeeds(vx, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return !lidar.isConnected() || (!Double.isNaN(error) && Math.abs(error) < TOLERANCE);
  }

  @Override
  public void end(boolean interrupted) {
    drive.accept(new ChassisSpeeds());
  }
}

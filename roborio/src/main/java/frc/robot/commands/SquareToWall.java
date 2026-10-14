package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LidarSubsystem;
import java.util.function.Consumer;

/**
 * Rotates in place until the robot faces the wall ahead squarely.
 *
 * <p>The lidar fits a straight line to the points in front of the robot. The angle of that line
 * tells us how far the robot is turned away from square, and we turn the opposite way.
 *
 * <p>Ends when square within tolerance, or if no wall is visible.
 */
public class SquareToWall extends Command {
  private static final double KP = 3.0; // (rad/s) per radian of error
  private static final double MAX_OMEGA = 1.5; // rad/s
  private static final double TOLERANCE_RAD = Math.toRadians(1.0);

  private final LidarSubsystem lidar;
  private final Consumer<ChassisSpeeds> drive;
  private double error = Double.NaN;

  public SquareToWall(
      LidarSubsystem lidar, Subsystem driveSubsystem, Consumer<ChassisSpeeds> robotRelativeDrive) {
    this.lidar = lidar;
    this.drive = robotRelativeDrive;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    error = Double.NaN;
  }

  @Override
  public void execute() {
    var wall = lidar.getWall();
    if (wall.isEmpty()) {
      error = Double.NaN;
      drive.accept(new ChassisSpeeds());
      return;
    }
    // Positive angle means the robot is turned counterclockwise, so turn clockwise (negative)
    error = wall.get().angle().getRadians();
    double omega = MathUtil.clamp(-KP * error, -MAX_OMEGA, MAX_OMEGA);
    drive.accept(new ChassisSpeeds(0.0, 0.0, omega));
  }

  @Override
  public boolean isFinished() {
    return lidar.getWall().isEmpty() || (!Double.isNaN(error) && Math.abs(error) < TOLERANCE_RAD);
  }

  @Override
  public void end(boolean interrupted) {
    drive.accept(new ChassisSpeeds());
  }
}

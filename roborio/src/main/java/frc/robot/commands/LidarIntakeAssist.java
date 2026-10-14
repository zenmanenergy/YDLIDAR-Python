package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.LidarSubsystem;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Consumer;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

/**
 * Drives to a game piece: PhotonVision steers from far away, the lidar takes over up close.
 *
 * <p>Far away, the camera's object detection gives the direction to the piece and the robot
 * drives toward it at cruise speed. Once the lidar sees something along that same direction
 * (inside its ~300 mm range), the lidar's distance controls the final approach and the command
 * stops at an exact gap.
 *
 * <p>The lidar only sees objects at the height it is mounted. Mount it low enough to hit the
 * game piece, or this command will only ever use the camera.
 */
public class LidarIntakeAssist extends Command {
  private static final double CRUISE_SPEED = 1.0; // m/s while only the camera sees the piece
  private static final double APPROACH_KP = 3.0; // (m/s) per meter, once the lidar has it
  private static final double MAX_APPROACH_SPEED = 0.6; // m/s
  private static final double TURN_KP = 3.0; // (rad/s) per radian
  private static final double MAX_OMEGA = 2.0; // rad/s
  private static final double CORRIDOR_HALF_WIDTH = 0.08; // meters either side of the camera ray
  private static final double TARGET_TIMEOUT = 0.5; // seconds without a camera sighting
  private static final double STOP_TOLERANCE = 0.015; // meters

  private final LidarSubsystem lidar;
  private final PhotonCamera camera;
  private final Transform2d robotToCamera;
  private final Consumer<ChassisSpeeds> drive;
  private final double stopDistanceMeters;

  private Rotation2d rayDirection = null; // robot-relative direction to the piece, from the camera
  private double lastSeen = Double.NEGATIVE_INFINITY;
  private OptionalDouble lidarDistance = OptionalDouble.empty();

  /**
   * @param lidar the lidar subsystem
   * @param camera a PhotonVision camera running an object detection (or color) pipeline
   * @param robotToCamera where the camera is on the robot (x, y, and the direction it faces)
   * @param driveSubsystem your drivetrain
   * @param robotRelativeDrive drives the robot with robot-relative speeds
   * @param stopDistanceMeters gap between the camera position and the piece at which to stop.
   *     Tune this so the piece ends up in the intake.
   */
  public LidarIntakeAssist(
      LidarSubsystem lidar,
      PhotonCamera camera,
      Transform2d robotToCamera,
      Subsystem driveSubsystem,
      Consumer<ChassisSpeeds> robotRelativeDrive,
      double stopDistanceMeters) {
    this.lidar = lidar;
    this.camera = camera;
    this.robotToCamera = robotToCamera;
    this.drive = robotRelativeDrive;
    this.stopDistanceMeters = stopDistanceMeters;
    addRequirements(driveSubsystem);
  }

  @Override
  public void initialize() {
    rayDirection = null;
    lastSeen = Double.NEGATIVE_INFINITY;
    lidarDistance = OptionalDouble.empty();
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();

    // 1. Camera: which way is the piece? PhotonVision yaw is positive to the RIGHT,
    //    WPILib angles are positive to the LEFT, so negate it.
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) {
      PhotonPipelineResult latest = results.get(results.size() - 1);
      if (latest.hasTargets()) {
        double yawDeg = latest.getBestTarget().getYaw();
        rayDirection = robotToCamera.getRotation().plus(Rotation2d.fromDegrees(-yawDeg));
        lastSeen = now;
      }
    }

    if (rayDirection == null) {
      drive.accept(new ChassisSpeeds()); // never seen the piece yet
      return;
    }

    // 2. Lidar: is there something along that same line, close enough to measure precisely?
    Translation2d cameraPos = robotToCamera.getTranslation();
    Optional<Translation2d> piece =
        lidar.getClosestPointAlongRay(cameraPos, rayDirection, CORRIDOR_HALF_WIDTH);
    lidarDistance =
        piece.isPresent()
            ? OptionalDouble.of(piece.get().minus(cameraPos).getNorm())
            : OptionalDouble.empty();

    // Cameras often lose the piece right before it reaches the bumper. Keep going on the lidar
    // alone in that case, but stop if neither sensor has it.
    if (now - lastSeen > TARGET_TIMEOUT && piece.isEmpty()) {
      drive.accept(new ChassisSpeeds());
      return;
    }

    // Steer so the piece is straight ahead of the robot center. When the lidar has it, use the
    // measured point (exact), and re-aim the ray at it so it stays tracked as the robot turns.
    // Otherwise use the camera direction (good enough far away).
    Rotation2d bearing = rayDirection;
    if (piece.isPresent()) {
      rayDirection = piece.get().minus(cameraPos).getAngle();
      bearing = piece.get().getAngle();
    }
    double omega = MathUtil.clamp(TURN_KP * bearing.getRadians(), -MAX_OMEGA, MAX_OMEGA);

    // 3. Forward speed: cruise on camera, then hand off to the lidar for the final approach.
    double vx;
    if (lidarDistance.isPresent()) {
      double error = lidarDistance.getAsDouble() - stopDistanceMeters;
      vx = MathUtil.clamp(APPROACH_KP * error, 0.0, MAX_APPROACH_SPEED);
    } else {
      vx = CRUISE_SPEED;
    }
    // Slow down while still turning hard, so we don't sweep past the piece
    vx *= Math.max(0.0, Math.cos(bearing.getRadians()));

    drive.accept(new ChassisSpeeds(vx, 0.0, omega));
  }

  @Override
  public boolean isFinished() {
    boolean arrived =
        lidarDistance.isPresent()
            && lidarDistance.getAsDouble() <= stopDistanceMeters + STOP_TOLERANCE;
    boolean lost =
        rayDirection != null
            && Timer.getFPGATimestamp() - lastSeen > TARGET_TIMEOUT
            && lidarDistance.isEmpty();
    return arrived || lost;
  }

  @Override
  public void end(boolean interrupted) {
    drive.accept(new ChassisSpeeds());
  }
}

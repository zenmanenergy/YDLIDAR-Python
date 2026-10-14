package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;

/**
 * Reads the lidar data published by lidar.py on the Orange Pi (NetworkTables table "/lidar").
 *
 * <p>All coordinates are robot-relative, WPILib convention: origin at robot center, +x forward,
 * +y left, counterclockwise-positive angles, meters. Distances called "from the bumper" are
 * measured from the front bumper face, using the robot length set in config.json on the Pi.
 *
 * <p>Every getter returns an empty Optional when the data is stale, so if the Orange Pi script
 * stops or a lidar unplugs, commands see "no data" instead of an old number.
 */
public class LidarSubsystem extends SubsystemBase {
  /**
   * A flat surface in front of the robot.
   *
   * @param distanceMeters perpendicular distance from the front bumper to the wall
   * @param angle how far the robot is turned away from facing the wall squarely, counterclockwise
   *     positive. Turn by the negative of this to square up.
   */
  public record Wall(double distanceMeters, Rotation2d angle) {}

  /** Data older than this is treated as missing. */
  private static final double STALE_SECONDS = 0.25;

  private final IntegerSubscriber heartbeat;
  private final BooleanSubscriber connected;
  private final DoubleSubscriber latencyMs;
  private final DoubleSubscriber frontDistance;
  private final BooleanSubscriber wallValid;
  private final DoubleSubscriber wallDistance;
  private final DoubleSubscriber wallAngleDeg;
  private final DoubleArraySubscriber mergedPoints;

  private final Supplier<Pose2d> poseSupplier;
  private final StructArrayPublisher<Translation2d> fieldPointsPublisher;

  private long lastHeartbeat = Long.MIN_VALUE;
  private double lastHeartbeatChange = Double.NEGATIVE_INFINITY;
  private boolean fresh = false;
  private List<Translation2d> points = List.of();
  private double pointsTimestamp = 0.0;

  /** Lidar subsystem without field-relative output. */
  public LidarSubsystem() {
    this(null);
  }

  /**
   * @param poseSupplier the robot's field pose (for example from a pose estimator fed by
   *     PhotonVision). When given, lidar points are also published in field coordinates for
   *     AdvantageScope. May be null.
   */
  public LidarSubsystem(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("lidar");
    heartbeat = table.getIntegerTopic("heartbeat").subscribe(0);
    connected = table.getBooleanTopic("connected").subscribe(false);
    latencyMs = table.getDoubleTopic("latencyMs").subscribe(-1.0);
    frontDistance = table.getDoubleTopic("frontDistance").subscribe(-1.0);
    wallValid = table.getBooleanTopic("wall/valid").subscribe(false);
    wallDistance = table.getDoubleTopic("wall/distance").subscribe(0.0);
    wallAngleDeg = table.getDoubleTopic("wall/angleDeg").subscribe(0.0);
    mergedPoints = table.getDoubleArrayTopic("merged/points").subscribe(new double[0]);

    fieldPointsPublisher =
        poseSupplier == null
            ? null
            : inst.getStructArrayTopic("/Lidar/FieldPoints", Translation2d.struct).publish();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    // The Pi bumps the heartbeat every update. If it stops changing, the script is not running.
    long beat = heartbeat.get();
    if (beat != lastHeartbeat) {
      lastHeartbeat = beat;
      lastHeartbeatChange = now;
    }
    fresh = (now - lastHeartbeatChange) < STALE_SECONDS && connected.get();

    if (fresh) {
      TimestampedDoubleArray sample = mergedPoints.getAtomic();
      points = unflatten(sample.value);
      // NetworkTables timestamps on the robot use the FPGA clock (microseconds)
      pointsTimestamp = sample.timestamp / 1e6 - Math.max(latencyMs.get(), 0.0) / 1000.0;
    } else {
      points = List.of();
    }

    SmartDashboard.putBoolean("Lidar/Connected", fresh);
    SmartDashboard.putNumber("Lidar/FrontDistance", getFrontDistance().orElse(-1.0));
    SmartDashboard.putNumber("Lidar/PointCount", points.size());

    if (fieldPointsPublisher != null) {
      Pose2d pose = poseSupplier.get();
      fieldPointsPublisher.set(pose == null ? new Translation2d[0] : toField(points, pose));
    }
  }

  /** True when lidar data is arriving and at least one lidar is working. */
  public boolean isConnected() {
    return fresh;
  }

  /**
   * Distance from the front bumper to the closest object directly ahead (within the robot's
   * width), in meters. Empty if nothing is in range or the data is stale.
   */
  public OptionalDouble getFrontDistance() {
    if (!fresh) {
      return OptionalDouble.empty();
    }
    double d = frontDistance.get();
    return d < 0 ? OptionalDouble.empty() : OptionalDouble.of(d);
  }

  /**
   * The flat surface in front of the robot, if the points ahead form a clean straight line.
   * Distance is measured perpendicular to the wall, from the front bumper.
   */
  public Optional<Wall> getWall() {
    if (!fresh || !wallValid.get()) {
      return Optional.empty();
    }
    return Optional.of(new Wall(wallDistance.get(), Rotation2d.fromDegrees(wallAngleDeg.get())));
  }

  /** All current lidar points, robot-relative, in meters. Empty if stale. */
  public List<Translation2d> getPoints() {
    return points;
  }

  /**
   * FPGA timestamp (seconds) when the current points were measured, for latency compensation.
   * Compare with Timer.getFPGATimestamp().
   */
  public double getPointsTimestamp() {
    return pointsTimestamp;
  }

  /**
   * Distance along a ray to the closest lidar point inside a corridor around that ray.
   *
   * <p>Use this to ask "how far is the thing in that direction?" from any spot on the robot. For
   * example, pass a camera's position and the direction PhotonVision sees a game piece in, and get
   * back the lidar's precise distance to it.
   *
   * @param origin start of the ray, robot-relative, meters
   * @param direction direction of the ray, robot-relative
   * @param halfWidthMeters how far to either side of the ray a point may be and still count
   * @return distance from origin along the ray, or empty if no point is in the corridor
   */
  public OptionalDouble getDistanceAlongRay(
      Translation2d origin, Rotation2d direction, double halfWidthMeters) {
    double cos = direction.getCos();
    double sin = direction.getSin();
    return getClosestPointAlongRay(origin, direction, halfWidthMeters)
        .map(
            p ->
                OptionalDouble.of(
                    (p.getX() - origin.getX()) * cos + (p.getY() - origin.getY()) * sin))
        .orElse(OptionalDouble.empty());
  }

  /**
   * Same search as {@link #getDistanceAlongRay}, but returns the lidar point itself
   * (robot-relative). Useful for tracking an object as the robot turns.
   */
  public Optional<Translation2d> getClosestPointAlongRay(
      Translation2d origin, Rotation2d direction, double halfWidthMeters) {
    double cos = direction.getCos();
    double sin = direction.getSin();
    double best = Double.POSITIVE_INFINITY;
    Translation2d bestPoint = null;
    for (Translation2d p : points) {
      double vx = p.getX() - origin.getX();
      double vy = p.getY() - origin.getY();
      double along = vx * cos + vy * sin;
      double across = -vx * sin + vy * cos;
      if (along > 0 && Math.abs(across) <= halfWidthMeters && along < best) {
        best = along;
        bestPoint = p;
      }
    }
    return Optional.ofNullable(bestPoint);
  }

  /** Closest point within a corridor along a ray from the robot's center. */
  public OptionalDouble getDistanceAtBearing(Rotation2d bearing, double halfWidthMeters) {
    return getDistanceAlongRay(Translation2d.kZero, bearing, halfWidthMeters);
  }

  /** Converts robot-relative points to field coordinates using the robot's pose. */
  public static Translation2d[] toField(List<Translation2d> robotPoints, Pose2d robotPose) {
    Translation2d[] out = new Translation2d[robotPoints.size()];
    for (int i = 0; i < out.length; i++) {
      out[i] = robotPoints.get(i).rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation());
    }
    return out;
  }

  private static List<Translation2d> unflatten(double[] flat) {
    List<Translation2d> out = new ArrayList<>(flat.length / 2);
    for (int i = 0; i + 1 < flat.length; i += 2) {
      out.add(new Translation2d(flat[i], flat[i + 1]));
    }
    return out;
  }
}

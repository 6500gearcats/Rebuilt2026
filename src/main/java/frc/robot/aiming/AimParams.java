package frc.robot.aiming;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OnboardLogger;

/**
 * The parameters of a shooter at a particlar moment in time. This is the type that is returned when
 * a shot is generated, meaning that this is what is applied to each subystem.
 */
public class AimParams {
  /** The status of the parameters object. */
  public AimStatus status = AimStatus.Unchecked;
  /** The manner of speed control the paramters requires */
  public SpeedControl control = SpeedControl.ProjectileVelocity;
  /** the launch angle of the fuel out of the robot. */
  public Rotation2d pitch = Rotation2d.kZero;
  /**
   * the direction that the fuel should be ejected out at, relative to the robot (but still in the
   * field's coordinate system). In practice, this is basically the field-relative heading of the
   * shooter.
   */
  public Rotation2d yaw = Rotation2d.kZero;
  /**
   * the output that the fuel should be ejected out at. If control is ProjectileVelocity, then this
   * is m/s relative to the robot.
   */
  public double output = 0.0;
  /** the tolerated error in the shot's pitch */
  public Rotation2d deltaPitch = Rotation2d.fromDegrees(4);
  /** the tolerated error in the shot's yaw */
  public Rotation2d deltaYaw = Rotation2d.fromDegrees(4);
  /** the tolerated error in the shot's velocity */
  public double deltaOutput = 1.5;
  /** time-of-flight in seconds from launch to target; populated by the aim strategy */
  public double tof = 0.0;

  public AimParams() {}

  public AimParams(AimStatus status) {
    this.status = status;
  }

  public enum AimStatus {
    /** The program has not yet evaluated the valididty of this parameters object */
    Unchecked,
    /** Not a possible shot, do not try to attempt. Values are invalid. */
    Impossible,
    /** A shot that is calculated to go in */
    Possible;

    public boolean isOk() {
      return this == Possible;
    }
  }

  public static AimParams impossible() {
    return new AimParams(AimStatus.Impossible);
  }

  /** Returns whether the aim parameters calculated are feasible */
  public boolean isOk() {
    return status.isOk();
  }

  public enum SpeedControl {
    /**
     * The velocity parameter of the {@link AimParams} is the projectile's desired velocity, in
     * meters per second.
     */
    ProjectileVelocity,
    /**
     * The velocity parameter of the {@link AimParams} is the shooter's control input, in its
     * appropriate units.
     */
    MechanismControl;
  }

  public static void setupLogging(OnboardLogger log, Supplier<AimParams> params) {
    log.registerString("Status", () -> params.get().status.toString());
    log.registerMeasurement("Pitch", () -> params.get().pitch.getMeasure(), Degrees);
    log.registerMeasurement("Yaw", () -> params.get().yaw.getMeasure(), Degrees);
    log.registerDouble("Velocity", () -> params.get().output);
    log.registerMeasurement("Error/Pitch", () -> params.get().deltaPitch.getMeasure(), Degrees);
    log.registerMeasurement("Error/Yaw", () -> params.get().deltaYaw.getMeasure(), Degrees);
    log.registerDouble("Error/Velocity", () -> params.get().deltaOutput);
  }
}

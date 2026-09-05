package frc.robot.aiming;

import static edu.wpi.first.units.Units.Degrees;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.OnboardLogger;

/**
 * A snapshot of all computed shot parameters for a single aiming pipeline evaluation.
 *
 * <h2>Role in the Pipeline</h2>
 * The aiming pipeline produces an {@code AimParams} object each time an aim strategy is queried:
 * <pre>
 *   LeadCompensator → ToFAim.update() → AimParams
 * </pre>
 * This object is then consumed by:
 * <ul>
 *   <li>{@link frc.robot.subsystems.shooter.Shooter#shoot Shooter.shoot} — reads {@code pitch},
 *       {@code output}, and {@code control} to drive the hood and flywheel.
 *   <li>{@link frc.robot.subsystems.turret.Turret#track Turret.track} — reads {@code yaw} and
 *       {@code deltaYaw} to servo the turret toward the required field-relative heading.
 *   <li>{@link frc.robot.RobotStateMachine#isShootReady RobotStateMachine.isShootReady} — compares
 *       actual mechanism positions against the {@code delta*} tolerance fields to decide whether
 *       it is safe to fire.
 * </ul>
 *
 * <h2>Validity Model</h2>
 * Not every call to the pipeline produces a usable shot. Reasons a shot might be impossible:
 * target not in camera view, robot out of range, turret at a travel limit. The {@link #status}
 * field encodes validity. Callers should always check {@link #isOk()} before using numeric fields.
 * Use the {@link #impossible()} factory to create an empty, safe "no shot" sentinel.
 */
public class AimParams {
  /**
   * Validity of this params object. Callers check {@link #isOk()} before using numeric fields.
   * Defaults to {@link AimStatus#Unchecked} on construction.
   */
  public AimStatus status = AimStatus.Unchecked;

  /**
   * How to interpret the {@link #output} field. Either a desired ball exit speed in m/s, or a
   * raw mechanism RPS value from a calibrated lookup table.
   */
  public SpeedControl control = SpeedControl.ProjectileVelocity;

  /**
   * Required launch pitch angle, measured from horizontal in the robot frame.
   * The hood servo positions itself to achieve this launch angle.
   * Positive = more upward (high hub), negative = flatter (close shot).
   */
  public Rotation2d pitch = Rotation2d.kZero;

  /**
   * Required turret yaw, in the field coordinate frame.
   * The turret converts this to a mechanism angle using:
   * {@code mechanismAngle = fieldYaw − robotHeading + kForwards}.
   * Stored field-relative so it is unaffected by robot rotation between updates.
   */
  public Rotation2d yaw = Rotation2d.kZero;

  /**
   * Required launch speed. Interpreted as m/s ball velocity when
   * {@code control == ProjectileVelocity}, or as wheel RPS when
   * {@code control == MechanismControl}. Populated from the distance-to-speed lookup table
   * in {@link ToFAim}.
   */
  public double output = 0.0;

  /**
   * Maximum tolerated hood pitch error for a "ready to fire" signal. Default ±4°.
   * Tighter values improve accuracy; looser values allow shooting sooner.
   */
  public Rotation2d deltaPitch = Rotation2d.fromDegrees(4);

  /**
   * Maximum tolerated turret yaw error for a "ready to fire" signal. Default ±4°.
   * At 5 m range, 4° corresponds to roughly 35 cm of aim error at the hub — well within hub radius.
   */
  public Rotation2d deltaYaw = Rotation2d.fromDegrees(4);

  /**
   * Maximum tolerated flywheel speed error for a "ready to fire" signal, in the same units as
   * {@link #output}. Default 1.5 RPS (roughly 3% of full speed at typical FRC shooting speeds).
   */
  public double deltaOutput = 1.5;

  /**
   * Time-of-flight in seconds: elapsed time between ball leaving the shooter and reaching the hub.
   * Populated by {@link ToFAim#update} from the calibrated distance-to-time lookup table.
   * Used by {@link frc.robot.aiming.LeadCompensator LeadCompensator} to compute the forward lead
   * offset, and published as telemetry via {@code Aiming/LeadOffsetXM} and {@code Aiming/LeadOffsetYM}.
   */
  public double tof = 0.0;

  public AimParams() {}

  public AimParams(AimStatus status) {
    this.status = status;
  }

  /**
   * Validity state of an {@link AimParams} object.
   * Callers should check {@link AimParams#isOk()} rather than comparing status directly,
   * so that future additions to this enum do not break existing call sites.
   */
  public enum AimStatus {
    /** Pipeline has not evaluated this object yet — treat as invalid. */
    Unchecked,
    /** No valid shot is possible (no target, out of range, travel-limited). Numeric fields are zeros. */
    Impossible,
    /** A computable, in-range shot. Numeric fields are valid and should be applied to hardware. */
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
     * {@link AimParams#output} is ball exit velocity in m/s. Shooter converts to wheel RPS using
     * {@code kMaxRotationalSpeed × (output / kMaxLinearSpeed)}.
     */
    ProjectileVelocity,
    /**
     * {@link AimParams#output} is a direct wheel speed in RPS — used when {@link ToFAim} already
     * consulted the calibrated lookup table and wants to command the mechanism speed directly.
     */
    MechanismControl;
  }

  /**
   * Registers all aim parameter fields with an {@link OnboardLogger} under the {@code "Aiming/"}
   * namespace. This is a static registration — each supplier is evaluated every time the logger
   * flushes, so the telemetry always reflects the current pipeline output without requiring the
   * caller to push updates manually.
   *
   * <p>Called once in {@link frc.robot.RobotStateMachine} constructor. All subsequent writes to
   * SmartDashboard for aiming telemetry go through this logger — no individual {@code put*} calls.
   *
   * @param log    the logger instance scoped to the {@code "Aiming"} namespace
   * @param params supplier of the current {@code AimParams} object (called each flush)
   */
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

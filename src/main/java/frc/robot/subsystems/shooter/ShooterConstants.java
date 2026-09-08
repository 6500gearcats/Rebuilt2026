package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.aiming.AimMeasurement;

/**
 * Hardware constants and calibration data for the shooter mechanism.
 *
 * <p>This class groups three categories of configuration:
 * <ul>
 *   <li><b>Flywheel constants</b> — CAN IDs, PID gains, current limits, and error thresholds
 *       for the two shooter motors. CAN IDs are set in Stage 8.
 *   <li><b>Hood constants</b> ({@link HoodConstants}) — CAN IDs, gear ratio, PID gains,
 *       software limits, and encoder config for the hood angle motor. CAN IDs and magnet
 *       offset set in Stage 8.
 *   <li><b>Calibration tables</b> ({@link #scoringMeasurements}, {@link #feedingMeasurements})
 *       — empirical shot data collected from Hackbots; must be re-validated in Stage 8-8
 *       with the actual Gearcats robot.
 * </ul>
 */
public final class ShooterConstants {
  // CAN IDs — TBD at wiring (Stage 8)
  protected static final int kMotor1Id = 0;
  protected static final int kMotor2Id = 0;

  /**
   * Speed error (RPS below target) that triggers automatic switch to recovery control mode.
   * If the flywheel drops 8 RPS below its setpoint (e.g., after a note passes through),
   * {@link Shooter} switches to {@code VelocityDutyCycle} to recover faster at lower current.
   */
  protected static final AngularVelocity kRecoveryErrorThreshold = RotationsPerSecond.of(8);

  /**
   * Speed error (RPS) used to detect when a note has passed through the shooter.
   * A 4 RPS dip signals a shot event for logging and recovery logic.
   */
  protected static final AngularVelocity kShootingErrorDetectionThreshold = RotationsPerSecond.of(4);

  /**
   * TalonFX configuration applied to both flywheel motors.
   *
   * <p>{@code PeakReverseDutyCycle(0)} prevents the motors from spinning backward,
   * which would eject notes back into the hopper. The shooter is designed to spin in
   * one direction only — all "reverse" operations are handled by reducing speed, not
   * by reversing direction.
   */
  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withSlot0(new Slot0Configs()
          .withKP(3414)
          .withKI(0)
          .withKD(0))

      .withMotionMagic(new MotionMagicConfigs()
          .withMotionMagicAcceleration(10.0))

      .withMotorOutput(new MotorOutputConfigs()
          .withPeakReverseDutyCycle(0)
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.Clockwise_Positive))

      .withTorqueCurrent(new TorqueCurrentConfigs()
          .withPeakReverseTorqueCurrent(0.0)
          .withPeakForwardTorqueCurrent(100.0))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withStatorCurrentLimitEnable(true)
          .withSupplyCurrentLimit(100)
          .withStatorCurrentLimit(100));

  /** Radius of the shooter wheels, used to convert RPS to linear surface speed. */
  public static final Distance kRadius = Inches.of(2);

  /**
   * Flywheel speed used when reversing to unjam a note.
   * Applied only during explicit unjam commands, not during normal shooting.
   */
  public static final AngularVelocity kReverseVelocity = RotationsPerSecond.of(30.0);

  /**
   * Alignment value for motor 2.
   * {@link MotorAlignmentValue#Aligned} means motor 2 rotates in the same direction
   * as motor 1 mechanically (before any software inversion).
   */
  public static final MotorAlignmentValue kMotor2Alignment = MotorAlignmentValue.Aligned;

  /** Physical limit of linear ball speed at the shooter wheel surface. */
  public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(15.5);

  /** Physical limit of flywheel speed in rotations per second. */
  public static final AngularVelocity kMaxRotationalSpeed = RotationsPerSecond.of(100.0);

  /**
   * Constants for the hood mechanism, which adjusts the launch angle.
   *
   * <p>The hood is driven by a single TalonFX motor with a 155:15 gear reduction.
   * Position is measured by a CANcoder (absolute, for calibration) and the motor's
   * integrated encoder (fast, for closed-loop control after {@code calibrate()} runs).
   *
   * <p>Software limits are 0.0–0.065 rotations (hood-axis). The CANcoder magnet
   * offset is TBD until the hood is physically assembled and zeroed in Stage 8-3.
   */
  public static final class HoodConstants {
    // CAN IDs — TBD at wiring (Stage 8)
    protected static final int kMotorID = 0;
    protected static final int kCANcoderId = 0;

    protected static final double kRatio = 0.0028888888888;

    protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withFeedbackRemoteSensorID(kCANcoderId)
            .withSensorToMechanismRatio(155.0 / 15.0))

        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(0.065)
            .withReverseSoftLimitThreshold(0.0))

        .withSlot0(new Slot0Configs()
            .withKA(0.1)
            .withKS(0.3)
            .withKV(7)
            .withKP(60.0)
            .withKI(0)
            .withKD(0))

        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(3.0)
            .withMotionMagicAcceleration(4))

        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.CounterClockwise_Positive))

        .withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withStatorCurrentLimit(125));

    protected static final CANcoderConfiguration kCANcoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0.8)
            .withMagnetOffset(-0.150146484375)); // TBD Stage 8-3

    protected static final int kSlot = 0;

    /**
     * Physical angle of the hood when the encoder reports zero.
     * Accounts for the mechanical offset between the encoder's zero position
     * and the hood's true flat/closed position.
     */
    protected static final Angle kOffset = Degrees.of(18.0);
  }

  /**
   * Empirical aim calibration table for scoring into the hub.
   *
   * <p>Each entry maps a measured robot-to-hub distance to the hood angle, flywheel RPS,
   * and time-of-flight that produced a successful score during Hackbots testing.
   * The table is interpolated by {@link frc.robot.aiming.ToFAim} at runtime.
   *
   * <p><b>Must be re-validated in Stage 8-8</b> — the Hackbots data may not
   * transfer exactly to the Gearcats robot due to mechanical differences.
   */
  public static final List<AimMeasurement> scoringMeasurements = List.of(
      new AimMeasurement(Meters.of(1.70), Rotation2d.fromDegrees(72), 28.25, Seconds.of(0.962)),
      new AimMeasurement(Meters.of(2.41), Rotation2d.fromDegrees(70), 31.25, Seconds.of(1.006)),
      new AimMeasurement(Meters.of(2.93), Rotation2d.fromDegrees(67), 32.25, Seconds.of(1.016)),
      new AimMeasurement(Meters.of(3.33), Rotation2d.fromDegrees(65), 34.25, Seconds.of(1.014)),
      new AimMeasurement(Meters.of(3.81), Rotation2d.fromDegrees(61), 35.75, Seconds.of(1.07)),
      new AimMeasurement(Meters.of(4.23), Rotation2d.fromDegrees(60), 36.75, Seconds.of(1.0325)),
      new AimMeasurement(Meters.of(4.77), Rotation2d.fromDegrees(59), 39.75, Seconds.of(1.122)),
      new AimMeasurement(Meters.of(5.26), Rotation2d.fromDegrees(58), 40.25, Seconds.of(1.136)),
      new AimMeasurement(Meters.of(5.73), Rotation2d.fromDegrees(55), 41.25, Seconds.of(1.15)),
      new AimMeasurement(Meters.of(6.22), Rotation2d.fromDegrees(55), 43.25, Seconds.of(1.202)),
      new AimMeasurement(Meters.of(6.84), Rotation2d.fromDegrees(53), 45.5,  Seconds.of(1.265)));

  /**
   * Empirical aim calibration table for feeding notes to a partner robot (passing).
   *
   * <p>Same structure as {@link #scoringMeasurements} but tuned for the lower-arc
   * trajectory used when passing. Includes a 15 m lob entry (80 RPS) for full-field
   * passes. Also requires re-validation in Stage 8-8.
   */
  public static final List<AimMeasurement> feedingMeasurements = List.of(
      new AimMeasurement(Meters.of(1.70), Rotation2d.fromDegrees(72), 29,    Seconds.of(0)),
      new AimMeasurement(Meters.of(2.41), Rotation2d.fromDegrees(70), 32,    Seconds.of(1.006)),
      new AimMeasurement(Meters.of(2.93), Rotation2d.fromDegrees(67), 33,    Seconds.of(1.016)),
      new AimMeasurement(Meters.of(3.33), Rotation2d.fromDegrees(65), 35,    Seconds.of(1.014)),
      new AimMeasurement(Meters.of(3.81), Rotation2d.fromDegrees(61), 36.5,  Seconds.of(1.07)),
      new AimMeasurement(Meters.of(4.23), Rotation2d.fromDegrees(60), 37.5,  Seconds.of(1.018)),
      new AimMeasurement(Meters.of(4.77), Rotation2d.fromDegrees(59), 40.5,  Seconds.of(0.924)),
      new AimMeasurement(Meters.of(5.26), Rotation2d.fromDegrees(58), 41,    Seconds.of(1.136)),
      new AimMeasurement(Meters.of(5.73), Rotation2d.fromDegrees(55), 42,    Seconds.of(1.15)),
      new AimMeasurement(Meters.of(6.22), Rotation2d.fromDegrees(55), 44,    Seconds.of(1.126)),
      new AimMeasurement(Meters.of(6.84), Rotation2d.fromDegrees(53), 46.25, Seconds.of(1.134)),
      new AimMeasurement(Meters.of(15),   Rotation2d.fromDegrees(50), 80,    Seconds.of(2)));
}

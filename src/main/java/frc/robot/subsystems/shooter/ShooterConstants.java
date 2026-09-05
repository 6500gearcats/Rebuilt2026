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

public final class ShooterConstants {
  // CAN IDs — TBD at wiring (Stage 8)
  protected static final int kMotor1Id = 0;
  protected static final int kMotor2Id = 0;

  protected static final AngularVelocity kRecoveryErrorThreshold = RotationsPerSecond.of(8);
  protected static final AngularVelocity kShootingErrorDetectionThreshold = RotationsPerSecond.of(4);

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

  public static final Distance kRadius = Inches.of(2);
  public static final AngularVelocity kReverseVelocity = RotationsPerSecond.of(30.0);

  public static final MotorAlignmentValue kMotor2Alignment = MotorAlignmentValue.Aligned;

  public static final LinearVelocity kMaxLinearSpeed = MetersPerSecond.of(15.5);
  public static final AngularVelocity kMaxRotationalSpeed = RotationsPerSecond.of(100.0);

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

    /** The position of the hood when its sensor reads zero */
    protected static final Angle kOffset = Degrees.of(18.0);
  }

  // Hackbots-measured TOF data — re-validate in Stage 8-8
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

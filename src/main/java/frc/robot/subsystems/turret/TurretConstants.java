package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.Rotations;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Angle;

/** Turret hardware constants. CAN IDs and encoder offset set in Stage 8. */
public class TurretConstants {
  // CAN IDs — TBD at wiring (Stage 8)
  protected static final int kMotorId = 0;
  protected static final int kEncoderId = 0;

  protected static final double kSupplyCurrentLimit = 100;

  protected static final Angle kHomePosition = Revolutions.of(0);
  /** Position reading of the encoder when the turret faces directly forward on the robot. */
  protected static final Angle kForwards = Revolutions.of(0);

  protected static final double kGearRatio = 30.0;
  protected static final double kMaxSpeed = 3.0;
  protected static final double kMaxAcceleration = 10;

  protected static final Angle kTolerance = Degrees.of(1);

  protected static final Angle kMinAngle = Rotations.of(-0.5);
  protected static final Angle kMaxAngle = Rotations.of(0.5);

  // Single absolute CANcoder — magnet offset set in Stage 8-3
  protected static final CANcoderConfiguration kEncoderConfig = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
          .withAbsoluteSensorDiscontinuityPoint(1.0)
          .withMagnetOffset(0.0)); // TBD Stage 8

  // Motor config — RemoteCANcoder is the single absolute encoder; encoder value = mechanism angle
  protected static final TalonFXConfiguration kMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Coast)
          .withInverted(InvertedValue.CounterClockwise_Positive))

      .withFeedback(new FeedbackConfigs()
          .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
          .withFeedbackRemoteSensorID(kEncoderId)
          .withSensorToMechanismRatio(1.0))

      .withCurrentLimits(new CurrentLimitsConfigs()
          .withSupplyCurrentLimitEnable(true)
          .withSupplyCurrentLimit(kSupplyCurrentLimit))

      .withSlot0(new Slot0Configs()
          .withKP(35)
          .withKI(0)
          .withKD(0.1)
          .withKS(0.6)
          .withKV(4.0)
          .withKA(0))

      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
          .withForwardSoftLimitEnable(true)
          .withForwardSoftLimitThreshold(kMaxAngle)
          .withReverseSoftLimitEnable(true)
          .withReverseSoftLimitThreshold(kMinAngle))

      .withMotionMagic(new MotionMagicConfigs()
          .withMotionMagicCruiseVelocity(kMaxSpeed)
          .withMotionMagicAcceleration(kMaxAcceleration));

  /** Turret base position relative to robot center — set from CAD in Stage 8-2 */
  public static final Transform3d kOffset = new Transform3d(
      -0.11, 0.11, 0.512, Rotation3d.kZero); // PLACEHOLDER from Hackbots — re-measure Stage 8
}

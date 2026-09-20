// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

/** Controls the turret hood and reports its absolute angle sensor. */
public class Hood extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(MotorConstants.kTurretHoodID);
  private final CANcoder m_encoder = new CANcoder(MotorConstants.kTurretHoodEncoderID);
  private double m_commandedSpeed = 0.0;

  public Hood() {
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(
                TurretConstants.kHoodEncoderDiscontinuityPointRotations)
            .withMagnetOffset(-TurretConstants.kHoodEncoderZeroRotations));
    m_encoder.getConfigurator().apply(encoderConfig);

    TalonFXConfiguration motorConfig = new TalonFXConfiguration()
            .withFeedback(new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(MotorConstants.kTurretHoodEncoderID)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
                    // The CANcoder is directly measuring the hood mechanism.
                    .withSensorToMechanismRatio(1.0))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(
                            TurretConstants.kHoodMaxPositionRotations)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(
                            TurretConstants.kHoodMinPositionRotations));

    m_motor.getConfigurator().apply(motorConfig);
  }

  @Override
  public void periodic() {
    var positionSignal = m_encoder.getAbsolutePosition();
    double absolutePositionRotations = positionSignal.getValueAsDouble();
    boolean encoderConnected = positionSignal.getStatus().isOK();

    if (!encoderConnected || isMotionBlocked(m_commandedSpeed, absolutePositionRotations)) {
      stop();
    }

    // Disabled high-rate SmartDashboard telemetry: reported hood position, encoder connectivity,
    // and lower/upper software-limit state.
    // SmartDashboard.putNumber("Hood Absolute Position (rotations)", absolutePositionRotations);
    // SmartDashboard.putNumber("Hood Absolute Position (degrees)", absolutePositionRotations * 360.0);
    // SmartDashboard.putBoolean("Hood Encoder Connected", encoderConnected);
    // SmartDashboard.putBoolean("Hood At Lower Limit", isAtLowerLimit(absolutePositionRotations));
    // SmartDashboard.putBoolean("Hood At Upper Limit", isAtUpperLimit(absolutePositionRotations));
  }

  /** Runs the hood motor at the requested duty cycle. */
  public void setSpeed(double speed) {
    var positionSignal = m_encoder.getAbsolutePosition();
    double limitedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    if (!positionSignal.getStatus().isOK()
        || isMotionBlocked(limitedSpeed, positionSignal.getValueAsDouble())) {
      limitedSpeed = 0.0;
    }

    m_commandedSpeed = limitedSpeed;
    m_motor.set(limitedSpeed);
  }

  /** Stops the hood motor. */
  public void stop() {
    m_commandedSpeed = 0.0;
    m_motor.stopMotor();
  }

  private boolean isMotionBlocked(double speed, double positionRotations) {
    return (speed < 0.0 && isAtLowerLimit(positionRotations))
        || (speed > 0.0 && isAtUpperLimit(positionRotations));
  }

  private boolean isAtLowerLimit(double positionRotations) {
    return positionRotations <= TurretConstants.kHoodMinPositionRotations;
  }

  private boolean isAtUpperLimit(double positionRotations) {
    return positionRotations >= TurretConstants.kHoodMaxPositionRotations;
  }

  /** Returns the zeroed absolute hood position in rotations. */
  public double getAbsolutePositionRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /** Returns the zeroed absolute hood position in degrees. */
  public double getAbsolutePositionDegrees() {
    return getAbsolutePositionRotations() * 360.0;
  }
}

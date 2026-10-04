// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see the WPILib BSD license file in this project.
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.utility.shooting.SetpointReadiness;

/** Holds hood CANcoder rotations in the same zeroed coordinate used by the shot table. */
public class Hood extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(MotorConstants.kTurretHoodID);
  private final CANcoder m_encoder = new CANcoder(MotorConstants.kTurretHoodEncoderID);
  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  private final SetpointReadiness readiness = new SetpointReadiness(0.005, 0.10);
  private final boolean configured;
  private boolean initialized;
  private boolean holdingPosition;
  private double targetPosition = Double.NaN;

  public Hood() {
    var encoderConfig = new CANcoderConfiguration().withMagnetSensor(new MagnetSensorConfigs()
        .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
        .withAbsoluteSensorDiscontinuityPoint(TurretConstants.kHoodEncoderDiscontinuityPointRotations)
        .withMagnetOffset(-TurretConstants.kHoodEncoderZeroRotations));
    boolean encoderConfigured = m_encoder.getConfigurator().apply(encoderConfig).isOK();
    var motorConfig = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withFeedbackRemoteSensorID(m_encoder.getDeviceID())
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(1.0))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(TurretConstants.kHoodMaxPositionRotations)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(TurretConstants.kHoodMinPositionRotations))
        .withSlot0(new Slot0Configs().withKS(1.0).withKP(1.0));
    boolean motorConfigured = m_motor.getConfigurator().apply(motorConfig).isOK();
    configured = encoderConfigured && motorConfigured;
    initializeFromAbsolute();
    if (!configured) {
      DriverStation.reportError("Hood configuration failed; shooting disabled", false);
    }
  }

  /** Seed relative feedback with the actual absolute position, never a presumed endpoint. */
  private void initializeFromAbsolute() {
    var absolute = m_encoder.getAbsolutePosition().refresh();
    if (configured && absolute.getStatus().isOK() && validPosition(absolute.getValueAsDouble())) {
      targetPosition = absolute.getValueAsDouble();
      initialized = m_encoder.setPosition(targetPosition).isOK();
    }
  }

  @Override
  public void periodic() {
    // Recover initialization only while disabled, before any shooting command.
    if (!initialized && DriverStation.isDisabled()) { initializeFromAbsolute(); }
    double actual = getAbsolutePositionRotations();
    boolean healthy = isHealthy() && DriverStation.isEnabled();
    boolean feedbackAgrees = Math.abs(actual - m_motor.getPosition().getValueAsDouble()) <= 0.005;
    readiness.update(targetPosition, actual, healthy && holdingPosition && feedbackAgrees, Timer.getFPGATimestamp());
    if (!healthy) { stop(); }
    SmartDashboard.putNumber("Hood Absolute Position (rotations)", actual);
    SmartDashboard.putNumber("Hood Requested Position (rotations)", targetPosition);
    SmartDashboard.putNumber("Hood Feedback Position (rotations)", m_motor.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("Hood Encoder Connected", isHealthy());
    SmartDashboard.putBoolean("Hood Ready", isAtPosition());
    SmartDashboard.putBoolean("Hood At Lower Limit", actual <= TurretConstants.kHoodMinPositionRotations);
    SmartDashboard.putBoolean("Hood At Upper Limit", actual >= TurretConstants.kHoodMaxPositionRotations);
  }

  public static boolean validPosition(double rotations) {
    return Double.isFinite(rotations) && rotations >= TurretConstants.kHoodMinPositionRotations
        && rotations <= TurretConstants.kHoodMaxPositionRotations;
  }

  /** Invalid requests stop rather than silently clamping to a different trajectory. */
  public void setPositionRotations(double rotations) {
    if (!validPosition(rotations) || !isHealthy() || !DriverStation.isEnabled()) {
      stop();
      return;
    }
    if (!holdingPosition || Math.abs(rotations - targetPosition) > 0.005) { readiness.reset(); }
    targetPosition = rotations;
    holdingPosition = true;
    if (!m_motor.setControl(positionRequest.withPosition(rotations)).isOK()) { stop(); }
  }

  /** Communication and range checks; transient CAN sample lag must not stop a move. */
  public boolean isHealthy() {
    var absolute = m_encoder.getAbsolutePosition();
    var feedback = m_motor.getPosition();
    double position = absolute.getValueAsDouble();
    return configured && initialized && absolute.getStatus().isOK() && feedback.getStatus().isOK()
        && validPosition(position) && validPosition(feedback.getValueAsDouble());
  }

  public double getAbsolutePositionRotations() { return m_encoder.getAbsolutePosition().getValueAsDouble(); }
  public double getAbsolutePositionDegrees() { return getAbsolutePositionRotations() * 360.0; }
  public double getRequestedPositionRotations() { return targetPosition; }
  public boolean isAtPosition() {
    double actual = getAbsolutePositionRotations();
    return holdingPosition && isHealthy() && readiness.isReady()
        && Math.abs(actual - targetPosition) <= 0.005
        && Math.abs(actual - m_motor.getPosition().getValueAsDouble()) <= 0.005;
  }

  public void stop() {
    holdingPosition = false;
    readiness.reset();
    m_motor.stopMotor();
  }
}

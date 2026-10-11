// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotStateMachine;

/**
 * Turret subsystem that controls the yaw motor and tracks its position.
 */
public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_motor = new TalonFX(Constants.MotorConstants.kTurretYawMotorID);
  private final CANcoder m_encoder = new CANcoder(Constants.MotorConstants.kTurretEncoderID);
  private PositionVoltage m_request;
  private final DigitalInput m_switch = new DigitalInput(4);
  private final RobotStateMachine robotStateMachine;
  private boolean overridden = false;
  private boolean feedbackInitialized = false;
  TalonFXConfiguration talonFXConfigs;

  public Turret(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    m_request = new PositionVoltage(0).withSlot(2);
    talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.Feedback.FeedbackRemoteSensorID = m_encoder.getDeviceID();
    talonFXConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // One encoder revolution is one turret revolution; requests use turret rotations.
    talonFXConfigs.Feedback.SensorToMechanismRatio = 1.0;
    // Positive output previously decreased the encoder angle. Reverse the motor
    // so positive closed-loop output now increases the selected feedback.
    talonFXConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonFXConfigs.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        TurretConstants.kTurretMinAngleDegrees / 360.0;
    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    talonFXConfigs.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        TurretConstants.kTurretMaxAngleDegrees / 360.0;

    // The old sensor advanced 90 motor rotations per turret revolution.
    // Convert its gains to output per turret rotation (and rotation/s, rotation/s^2).
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.2;
    slot0Configs.kV = 450.0;
    slot0Configs.kA = 270.0;
    slot0Configs.kP = 270.0;
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 36.0;

    var slot1Configs = talonFXConfigs.Slot1;
    // Dashboard tuning values now use turret rotations, not motor rotations.
    slot1Configs.kS = 0.2;
    slot1Configs.kV = SmartDashboard.getNumber("kV", 0);
    slot1Configs.kA = SmartDashboard.getNumber("kA", 0);
    slot1Configs.kP = SmartDashboard.getNumber("kP", 0);
    slot1Configs.kI = 0; // no output for integrated error
    slot1Configs.kD = SmartDashboard.getNumber("kD", 0);

    // Existing tuned gains, converted from motor rotations to turret rotations.
    var slot2Configs = talonFXConfigs.Slot2;
    slot2Configs.kS = 0.20757;
    slot2Configs.kV = 9.306;
    slot2Configs.kA = 0.680157;
    slot2Configs.kP = 672.741;
    slot2Configs.kI = 0; // no output for integrated error
    slot2Configs.kD = 32.9094;

    m_encoder.getPosition().setUpdateFrequency(100.0);
    m_encoder.getVelocity().setUpdateFrequency(100.0);
    zeroMotorPosition();
  }

  @Override
  public void periodic() {
    double absolutePositionRotations = getAbsolutePositionRotations();
    SmartDashboard.putBoolean("switch on or off", m_switch.get());
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
    SmartDashboard.putNumber("Turret Absolute Position (rotations)", absolutePositionRotations);
    SmartDashboard.putNumber("Turret Absolute Position (degrees)", absolutePositionRotations * 360.0);
    SmartDashboard.putBoolean("Turret Encoder Connected", isEncoderFeedbackReady());
    SmartDashboard.putNumber("Robot Rot in Deg", robotStateMachine.getPose().getRotation().getDegrees());

    if (!isEncoderFeedbackReady()) {
      m_motor.stopMotor();
    }
  }

  public void setSpeed(double speed) {
    if (!isEncoderFeedbackReady()) {
      m_motor.stopMotor();
      return;
    }
    if (!overridden) {
      double angleDegrees = getAbsolutePositionDegrees();
      if ((angleDegrees <= TurretConstants.kTurretManualMinAngleDegrees && speed < 0)
          || (angleDegrees >= TurretConstants.kTurretManualMaxAngleDegrees && speed > 0)) {
        speed = 0;
      }
    }
    m_motor.set(speed);
  }

  public void toggleOverride() {
    overridden = !overridden;
  }

  /**
   * Returns the motor controller's selected feedback in turret rotations.
   *
   * @return turret encoder feedback position
   */
  public double getMotorPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  /** Returns the raw absolute turret encoder position in rotations. */
  public double getAbsolutePositionRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /** Returns the raw absolute turret encoder position in degrees. */
  public double getAbsolutePositionDegrees() {
    return getAbsolutePositionRotations() * 360.0;
  }

  /** Synchronizes continuous feedback with the existing absolute encoder reference. */
  public void zeroMotorPosition() {
    m_motor.stopMotor();
    feedbackInitialized = false;
    var status = m_motor.getConfigurator().apply(talonFXConfigs);
    if (!status.isOK()) {
      DriverStation.reportError("Failed to configure turret encoder feedback: " + status, false);
      return;
    }
    var absolutePosition = m_encoder.getAbsolutePosition().waitForUpdate(0.5);
    if (!absolutePosition.getStatus().isOK()) {
      DriverStation.reportError("Failed to read turret absolute encoder: "
          + absolutePosition.getStatus(), false);
      return;
    }
    double rotations = absolutePosition.getValueAsDouble();
    var encoderStatus = m_encoder.setPosition(rotations);
    var motorStatus = m_motor.setPosition(rotations);
    feedbackInitialized = encoderStatus.isOK() && motorStatus.isOK();
    if (!feedbackInitialized) {
      DriverStation.reportError("Failed to synchronize turret feedback: encoder="
          + encoderStatus + ", motor=" + motorStatus, false);
    }
  }

  private boolean isEncoderFeedbackReady() {
    return feedbackInitialized && m_encoder.getAbsolutePosition().getStatus().isOK();
  }

  /**
   * Returns the turret position converted to degrees.
   *
   * @return turret angle in degrees
   */
  public double getConvertedTurretPosition() {
    return getAbsolutePositionDegrees();
  }

  /** Converts a turret angle in degrees to encoder rotations for position control. */
  public double unconvertPosition(double pos) {
    return pos / 360.0;
  }

  /**
   * Moves the turret to the given position setpoint.
   *
   * @param deg desired turret angle in degrees
   */
  public void setPosition(double deg) {
    if (robotStateMachine.ductTapeCorrection) {
      deg -= 5;
    }
    requestAngle(deg);
  }

  private void requestAngle(double deg) {
    if (!isEncoderFeedbackReady() || !Double.isFinite(deg)) {
      m_motor.stopMotor();
      return;
    }
    deg = MathUtil.clamp(deg, TurretConstants.kTurretMinAngleDegrees,
        TurretConstants.kTurretMaxAngleDegrees);
    SmartDashboard.putNumber("UnconvPos", unconvertPosition(deg));
    m_motor.setControl(m_request.withPosition(unconvertPosition(deg)));
  }

  /*
   * Gets turret speed in encoder rotations per second.
   */
  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public void goToZero() {
    requestAngle(0.0);
  }

  public void updateSlotConfigs() {
    var slot = talonFXConfigs.Slot1;
    slot.kV = SmartDashboard.getNumber("kV", 0);
    slot.kA = SmartDashboard.getNumber("kA", 0);
    slot.kP = SmartDashboard.getNumber("kP", 0);
    slot.kI = SmartDashboard.getNumber("kI", 0);
    slot.kD = SmartDashboard.getNumber("kD", 0);
    m_motor.getConfigurator().apply(slot);
    m_request = new PositionVoltage(0).withSlot(1);
  }

  public void setControl(ControlRequest req) {
    if (!isEncoderFeedbackReady()) {
      m_motor.stopMotor();
      return;
    }
    m_motor.setControl(req);
  }
}

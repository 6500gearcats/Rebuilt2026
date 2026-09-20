// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;

/**
 * Turret subsystem that controls the yaw motor and tracks its position.
 */
public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_motor = new TalonFX(Constants.MotorConstants.kTurretYawMotorID);
  private final CANcoder m_encoder = new CANcoder(Constants.MotorConstants.kTurretEncoderID);
  private PositionVoltage m_request;
  private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get();
  private final RobotStateMachine robotStateMachine;
  // private double tagRot = 0 - tagPose.getRotation().getAngle();
  TalonFXConfiguration talonFXConfigs;

  public Turret(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    m_request = new PositionVoltage(0).withSlot(0);

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(
                Constants.TurretConstants.kTurretEncoderDiscontinuityPointRotations)
            .withMagnetOffset(-Constants.TurretConstants.kTurretEncoderZeroRotations));
    m_encoder.getConfigurator().apply(encoderConfig);

    talonFXConfigs = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs()
            .withFeedbackRemoteSensorID(Constants.MotorConstants.kTurretEncoderID)
            .withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
            .withSensorToMechanismRatio(1.0))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(
                Constants.TurretConstants.kTurretMaxPositionRotations)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(
                Constants.TurretConstants.kTurretMinPositionRotations));

    // Closed-loop gains intentionally remain unset until the new turret is tuned.

    m_motor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    var positionSignal = m_encoder.getAbsolutePosition();
    double absolutePositionRotations = positionSignal.getValueAsDouble();
    boolean encoderConnected = positionSignal.getStatus().isOK();

    if (!encoderConnected) {
      stop();
    }

    // Disabled high-rate SmartDashboard telemetry: reported turret motor/encoder positions,
    // encoder connectivity, software-limit state, and robot heading.
    // SmartDashboard.putNumber("Turret Feedback Position (rotations)", getMotorPosition());
    // SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
    // SmartDashboard.putNumber("Turret Absolute Position (rotations)", absolutePositionRotations);
    // SmartDashboard.putNumber("Turret Absolute Position (degrees)", absolutePositionRotations * 360.0);
    // SmartDashboard.putBoolean("Turret Encoder Connected", encoderConnected);
    // SmartDashboard.putBoolean("Turret At Left Limit", isAtLeftLimit(absolutePositionRotations));
    // SmartDashboard.putBoolean("Turret At Right Limit", isAtRightLimit(absolutePositionRotations));
    // SmartDashboard.putNumber("Robot Rot in Deg", robotStateMachine.getPose().getRotation().getDegrees());
  }

  public void setSpeed(double speed) {
    if (!m_encoder.getAbsolutePosition().getStatus().isOK()) {
      stop();
      return;
    }
    m_motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  /**
   * Returns the Talon's selected turret feedback position in rotations.
   *
   * @return motor sensor position
   */
  public double getMotorPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  /** Returns the zeroed absolute turret encoder position in rotations. */
  public double getAbsolutePositionRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /** Returns the zeroed absolute turret encoder position in degrees. */
  public double getAbsolutePositionDegrees() {
    return getAbsolutePositionRotations() * 360.0;
  }

  /**
   * Returns turret angle in degrees, with left positive and right negative.
   *
   * @return turret angle in degrees
   */
  public double getConvertedTurretPosition() {
    return -getMotorPosition() * 360.0;
  }

  public double unconvertPosition(double pos) {
    return -pos / 360.0;
  }

  /**
   * Moves the turret to the given position setpoint.
   *
   * @param deg desired position in degress
   */
  public void setPosition(double deg) {
    if (!m_encoder.getAbsolutePosition().getStatus().isOK()) {
      stop();
      return;
    }
    if (robotStateMachine.ductTapeCorrection) {
      deg -= 5;
    }
    deg = MathUtil.clamp(
        deg,
        Constants.TurretConstants.kTurretMinAngleDegrees,
        Constants.TurretConstants.kTurretMaxAngleDegrees);
    // Disabled SmartDashboard telemetry: reported the requested turret setpoint in motor rotations.
    // SmartDashboard.putNumber("UnconvPos", unconvertPosition(deg));
    m_motor.setControl(m_request.withPosition(unconvertPosition(deg)));
  }

  /*
   * Gets Speed in RPS
   */
  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public void goToZero() {
    setPosition(0.0);
  }

  public void stop() {
    m_motor.stopMotor();
  }

  private boolean isAtLeftLimit(double positionRotations) {
    return positionRotations <= Constants.TurretConstants.kTurretMinPositionRotations;
  }

  private boolean isAtRightLimit(double positionRotations) {
    return positionRotations >= Constants.TurretConstants.kTurretMaxPositionRotations;
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
    if (m_encoder.getAbsolutePosition().getStatus().isOK()) {
      m_motor.setControl(req);
    } else {
      stop();
    }
  }
}

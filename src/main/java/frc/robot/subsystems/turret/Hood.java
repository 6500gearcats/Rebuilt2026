// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotStateMachine;
import frc.robot.DashboardTelemetry;
import frc.robot.generated.TunerConstants;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;

/** Controls the turret hood and reports its absolute angle sensor. */
public class Hood extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(MotorConstants.kTurretHoodID);
  private final CANcoder m_encoder = new CANcoder(MotorConstants.kTurretHoodEncoderID);
  private double m_commandedSpeed = 0.0;
  private double targetPosition = 0.0;
  private double HOOD_INCREMENT = 0.034;
  private RobotStateMachine robotStateMachine;
  private final DashboardTelemetry.DebugPublisher debugTelemetry = new DashboardTelemetry.DebugPublisher();

  private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);
  public Hood(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;

    CANcoderConfiguration encoderConfig = new CANcoderConfiguration()
        .withMagnetSensor(new MagnetSensorConfigs()
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
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
                            TurretConstants.kHoodMinPositionRotations)
//                            ).withSlot0(new Slot0Configs().withKS(1.0).withKP(1.0));
                            ).withSlot0(new Slot0Configs().withKS(2.0).withKV(0.02).withKP(10.0));
    m_motor.getConfigurator().apply(motorConfig);

    // Seed motor feedback and the first target from the calibrated absolute angle.
    var absolutePositionSignal = m_encoder.getAbsolutePosition().waitForUpdate(0.5);
    if (absolutePositionSignal.getStatus().isOK()) {
      targetPosition = absolutePositionSignal.getValueAsDouble();
      m_motor.setPosition(targetPosition);
    } else {
      targetPosition = m_motor.getPosition().getValueAsDouble();
      DriverStation.reportError("Failed to initialize hood position from absolute encoder: "
          + absolutePositionSignal.getStatus(), false);
    }
  }

  @Override
  public void periodic() {
    RobotStateMachine.ShotSolution shotSolution = robotStateMachine.getShotSolution();

    var positionSignal = m_encoder.getAbsolutePosition();
    double absolutePositionRotations = positionSignal.getValueAsDouble();
    // double posSignal = m_encoder.getPosition().getValueAsDouble();
    boolean encoderConnected = positionSignal.getStatus().isOK();

    if (!encoderConnected || isMotionBlocked(m_commandedSpeed, absolutePositionRotations)) {
      stop();
    }
    m_motor.setControl(positionRequest.withPosition(shotSolution.getHoodRotation()));

    publishTelemetry(absolutePositionRotations, encoderConnected, shotSolution.getHoodRotation());
  }

  private void publishTelemetry(double absolutePositionRotations, boolean encoderConnected,
      double requestedPositionRotations) {
    SmartDashboard.putNumber("Hood Absolute Position (degrees)", absolutePositionRotations * 360.0);
    SmartDashboard.putNumber("Hood Requested Position (degrees)", requestedPositionRotations * 360.0);
    SmartDashboard.putBoolean("Hood Encoder Connected", encoderConnected);
    SmartDashboard.putBoolean("Hood At Lower Limit", isAtLowerLimit(absolutePositionRotations));
    SmartDashboard.putBoolean("Hood At Upper Limit", isAtUpperLimit(absolutePositionRotations));
    if (debugTelemetry.shouldPublish()) {
      SmartDashboard.putNumber("Debug/Hood/Absolute Position (rotations)", absolutePositionRotations);
      SmartDashboard.putNumber("Debug/Hood/Manual Step Target (rotations)", targetPosition);
      SmartDashboard.putNumber("Debug/Hood/Manual Duty Cycle", m_commandedSpeed);
    }
  }

  /** Runs the hood motor at the requested duty cycle. */
  public void setSpeed(double speed) {
    // var positionSignal = m_encoder.getAbsolutePosition();
    double limitedSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    // if (!positionSignal.getStatus().isOK()
    //     || isMotionBlocked(limitedSpeed, positionSignal.getValueAsDouble())) {
    //   limitedSpeed = 0.0;
    // }

    m_commandedSpeed = limitedSpeed;
    m_motor.set(limitedSpeed);
  }

  /** Stops the hood motor. */
  public void stop() {
    m_commandedSpeed = 0.0;
    m_motor.stopMotor();
  }

  private boolean isMotionBlocked(double speed, double positionRotations) {
    // return (speed < 0.0 && isAtLowerLimit(positionRotations))
    //     || (speed > 0.0 && isAtUpperLimit(positionRotations));
    return false;
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

  public void moveDownOneStep(){
    targetPosition -= HOOD_INCREMENT;
    if (targetPosition <= TurretConstants.kHoodMinPositionRotations){
        targetPosition = TurretConstants.kHoodMinPositionRotations;
    }
    m_motor.setControl(positionRequest.withPosition(targetPosition));
    SmartDashboard.putNumber("Hood Requested Position (degrees)", targetPosition * 360.0);
  }
  public void moveUpOneStep(){
    targetPosition += HOOD_INCREMENT;
    if (targetPosition >= TurretConstants.kHoodMaxPositionRotations){
        targetPosition = TurretConstants.kHoodMaxPositionRotations;
    }
    m_motor.setControl(positionRequest.withPosition(targetPosition));
    SmartDashboard.putNumber("Hood Requested Position (degrees)", targetPosition * 360.0);
  }
};

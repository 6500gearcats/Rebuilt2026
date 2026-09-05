// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.Constants.MotorConstants;

/**
 * Flywheel subsystem that controls the shooter motors.
 */
public class Flywheel extends SubsystemBase {
  TalonFX m_motor = new TalonFX(Constants.MotorConstants.kShooterMotorRightID);
  TalonFX m_motor2 = new TalonFX(Constants.MotorConstants.kShooterMotorLeftID);
  VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  public boolean snurboEnable = false;
  public double speedModifier = 1;
  public boolean waitForSpeed = false;
  private double speedMultiplier = 0;
  public double rotationMultiplier = 0;
  private double reqSpeed;

  private RobotStateMachine robotStateMachine;
  TalonFXConfiguration talonFXConfigs;
  private final Timer m_telemetryTimer = new Timer();

  public Flywheel(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    talonFXConfigs = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.6));

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.3087;
    slot0Configs.kV = 0.076456;
    slot0Configs.kA = 0.010904;
    slot0Configs.kP = 0.026467;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor2.getConfigurator().apply(talonFXConfigs);
    m_telemetryTimer.start();
  }

  @Override
  public void periodic() {
    speedModifier = snurboEnable ? 0.15 : 1;

    if (m_telemetryTimer.advanceIfElapsed(0.1)) {
      SmartDashboard.putNumber("Flywheel/VelocityRPS",         m_motor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Flywheel/ReqVelocityRPS",      reqSpeed);
      SmartDashboard.putNumber("Flywheel/ActualVelocityRPS",   getSpeed());
      SmartDashboard.putNumber("Flywheel/SpeedMultiplier",     speedMultiplier);
      SmartDashboard.putNumber("Flywheel/RotationMultiplier",  rotationMultiplier);
      SmartDashboard.putBoolean("Flywheel/IsUpToSpeed",        isUpToSpeed());
      SmartDashboard.putBoolean("Flywheel/IsUnderTrench",      robotStateMachine.underTrench());
      SmartDashboard.putNumber("Flywheel/Motor1StatorCurrentA", m_motor.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Flywheel/Motor1SupplyVoltageV", m_motor.getSupplyVoltage().getValueAsDouble());
      SmartDashboard.putNumber("Flywheel/Motor1TempC",          m_motor.getDeviceTemp().getValueAsDouble());
      SmartDashboard.putNumber("Flywheel/Motor2StatorCurrentA", m_motor2.getStatorCurrent().getValueAsDouble());
      SmartDashboard.putNumber("Flywheel/Motor2TempC",          m_motor2.getDeviceTemp().getValueAsDouble());
    }
  }

  public void setSpeed(double speed) {
    reqSpeed = speed + (2 * speedMultiplier) + rotationMultiplier;

    SmartDashboard.putNumber("Flywheel/InitialSpeedRPS", speed);
    double speedValue = speed + (2 * speedMultiplier) + rotationMultiplier;
    if (speedValue > 0) {
      SmartDashboard.putNumber("Flywheel/AdjustedSpeedRPS", speedValue);

      if (robotStateMachine.underTrench()) {
        speedValue = 70 + (2 * speedMultiplier) + rotationMultiplier;
        reqSpeed = speedValue;
      }

      m_motor.setControl(m_request.withVelocity(speedValue));
      m_motor2.setControl(new Follower(MotorConstants.kShooterMotorRightID, MotorAlignmentValue.Opposed));
    }
  }

  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public double getReqSpeed() {
    return reqSpeed;
  }

  public boolean isUpToSpeed() {
    return Math.abs(reqSpeed - getSpeed()) < 2;
  }

  public void stopMotor() {
    m_motor.set(0);
    m_motor2.set(0);
  }

  public void incrementMultiplierUp() {
    speedMultiplier++;
  }

  public void incrementMultiplierDown() {
    speedMultiplier--;
  }

  public void setControl(ControlRequest req) {
    m_motor.setControl(req);
  }

  public void updateMotorConfigs() {
  }
}

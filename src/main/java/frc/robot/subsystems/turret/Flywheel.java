// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

/**
 * Flywheel subsystem that controls the shooter motors.
 */
public class Flywheel extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX m_motor = new TalonFX(Constants.MotorConstants.kShooterMotorRightID);
  VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  public boolean snurboEnable = false;
  public double speedModifier = 1;
  public boolean waitForSpeed = false;
  private double speedMultiplier = 0;
  private double reqSpeed;
  TalonFX m_motor2 = new TalonFX(Constants.MotorConstants.kShooterMotorLeftID);

  TalonFXConfiguration talonFXConfigs;

  // TODO: Add a constant Spin to the motors to not have to fight static friction

  public Flywheel() {

    talonFXConfigs = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.6));

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.31134; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.075575; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0064695; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.030814; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0;

    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor2.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    if (snurboEnable) {
      speedModifier = 0.15;
    } else {
      speedModifier = 1;
    }
    SmartDashboard.putNumber("Left Motor Speed", m_motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shot Multiplier", speedMultiplier);
    SmartDashboard.putBoolean("Up to Speed", isUpToSpeed());
    SmartDashboard.putNumber("reqSpeed", reqSpeed);
    SmartDashboard.putNumber("actSpeed", getSpeed());

    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {


    // set velocity to rps, add 0.5 V to overcome gravity
    SmartDashboard.putNumber("flywheel initial speed", speed);
    double speedValue = speed + (0.05 * speedMultiplier);
    if (speedValue > 0) {
      SmartDashboard.putNumber("flywheel sped-up speed", speedValue);

      m_motor.setControl(m_request.withVelocity(speedValue));
      m_motor2.setControl(new Follower(MotorConstants.kShooterMotorRightID, MotorAlignmentValue.Opposed));
    }
  }

  /*
   * Gets Speed in RPS
   */
  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
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
    // m_motor2.setControl(req);
  }

  public void updateMotorConfigs() {
    // var slot = talonFXConfigs.Slot0;
    // slot.kV = SmartDashboard.getNumber("shooter kV", 0);
    // slot.kA = SmartDashboard.getNumber("shooter kA", 0);
    // slot.kP = SmartDashboard.getNumber("shooter kP", 0);
    // slot.kI = SmartDashboard.getNumber("shooter kI", 0);
    // slot.kD = SmartDashboard.getNumber("shooter kD", 0);
    // m_motor.getConfigurator().apply(slot);
  }
}

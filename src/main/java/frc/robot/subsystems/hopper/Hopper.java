// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Hopper subsystem that controls the indexer and kicker motors.
 */
public class Hopper extends SubsystemBase {
  TalonFX m_hopperMotor = new TalonFX(Constants.MotorConstants.kIndexerMotorID);
  TalonFX m_kickerMotor = new TalonFX(Constants.MotorConstants.kKickerMotorID);

  public Hopper() {
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hopper/IndexerVelocityRPS",    m_hopperMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/IndexerStatorCurrentA", m_hopperMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/KickerVelocityRPS",     m_kickerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/KickerStatorCurrentA",  m_kickerMotor.getStatorCurrent().getValueAsDouble());
  }

  public void startAllMotors(double hopperSpeed, double kickerSpeed) {
    m_hopperMotor.set(hopperSpeed);
    m_kickerMotor.set(kickerSpeed);
  }

  public void stopAllMotors() {
    m_hopperMotor.set(0);
    m_kickerMotor.set(0);
  }

  public void setHopperSpeed(double speed) {
    m_hopperMotor.set(speed);
  }

  public void setKickerMotorSpeed(double speed) {
    m_kickerMotor.set(speed);
  }
}

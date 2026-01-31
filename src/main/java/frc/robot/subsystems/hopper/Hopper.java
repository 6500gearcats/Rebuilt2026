// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax m_hopperMotor = new SparkMax(Constants.MotorConstants.kIndexerMotorID, MotorType.kBrushless);
  SparkMax m_kickerMotor = new SparkMax(Constants.MotorConstants.kKickerMotorID, MotorType.kBrushless);

  public Hopper() {
  }

  @Override
  public void periodic() {
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
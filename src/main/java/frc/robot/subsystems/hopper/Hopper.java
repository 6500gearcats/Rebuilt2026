// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */
  SparkMax m_hopperMotor = new SparkMax(50, MotorType.kBrushless);
  SparkMax m_kickerFeeder = new SparkMax(51, MotorType.kBrushless);
  SparkMax m_kickerMotor = new SparkMax(52, MotorType.kBrushless);

  public Hopper() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_hopperMotor.set(0.1);
    m_kickerFeeder.set(0.1);
    m_kickerMotor.set(0.1);
  }
}

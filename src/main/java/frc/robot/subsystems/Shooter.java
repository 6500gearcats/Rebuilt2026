// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// make a spark max, with a specific id, make it spin

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  SparkMax m_shooterMotor;
  private double fake_speed;
  public Shooter() {
    m_shooterMotor = new SparkMax(1, MotorType.kBrushless);
  }

  public void shoot(double speed) {
    fake_speed = speed;
    m_shooterMotor.set(speed);
  }
  
  @Override
  public void periodic() {
      SmartDashboard.putNumber("Shooter Speed", fake_speed);

  }
}

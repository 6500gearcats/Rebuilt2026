// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Feeder extends SubsystemBase {
  /** Creates a new Feeder. */

  public SparkMax m_feedMotor;
  public SparkAbsoluteEncoder m_feedEncoder;

  public Feeder() {
    m_feedMotor = new SparkMax(10, MotorType.kBrushless); // Random DeviceID
    m_feedEncoder = m_feedMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("FeederSpeed", m_feedMotor.get());
    // This method will be called once per scheduler run
  }

  public void setFeedSpeed(double speed) {
    m_feedMotor.set(speed);
  }

  public double getFeedPosition() {
    return m_feedEncoder.getPosition();
  }

  public double getFeedVelocity() { // Did this one for fun (:
    return m_feedEncoder.getVelocity();
  }
}

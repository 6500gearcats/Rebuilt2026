// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new climber. */
  TalonFX m_motor = new TalonFX(Constants.ClimberConstants.kClimberMotorID);
  DigitalInput m_limitSwitch = new DigitalInput(Constants.ClimberConstants.kClimberLimitSwitchID);

  public Climber() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setMotorSpeed(double speed) {
    if (m_limitSwitch.get()) {
      m_motor.set(0);
    } else {
      m_motor.set(speed);
    }
  }

  public boolean getLimitSwitchStatus() {
    return m_limitSwitch.get();
  }

}

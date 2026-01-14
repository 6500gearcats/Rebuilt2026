// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//all below imports are from 2022 and may not work
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
  //all below code is from 2022 and may not work and all IntakeConstants have been changed to 1 or 2
  public final MotorController m_intakeMotor = new SparkMax(12, MotorType.kBrushed);

  public Intake() {}
  
  @Override
  public void simulationPeriodic() {
  // This method will be called once per scheduler run during simulation
  }
  
  public void setReverse(double speed) {
    m_intakeMotor.set(speed);
  }

  public void setPickupSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  public boolean ballIsPresent() {
    boolean ballIsPresent = false;
    //ballIsPresent = proxValue > 1.2;
    return ballIsPresent;
  }

  public void stop() {
    m_intakeMotor.stopMotor();
  }
}

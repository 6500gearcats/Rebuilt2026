// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX m_intakeMotor = new TalonFX(MotorConstants.kIntakeMotorID);
  private final TalonFX m_intakeDeployMotor = new TalonFX(MotorConstants.kIntakeDeployMotorID);
  public Intake() {}

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/DeployPositionRot",   m_intakeDeployMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerVelocityRPS",   m_intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/DeployStatorCurrentA", m_intakeDeployMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerStatorCurrentA", m_intakeMotor.getStatorCurrent().getValueAsDouble());
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }
  public void deployIntake(double speed){
    m_intakeDeployMotor.set(speed);
  }
}

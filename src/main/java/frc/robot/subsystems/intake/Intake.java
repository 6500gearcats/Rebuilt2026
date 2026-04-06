// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final TalonFX m_intakeMotor = new TalonFX(MotorConstants.kIntakeMotorID);
  private final TalonFX m_intakeDeployMotor = new TalonFX(MotorConstants.kIntakeDeployMotorID);

  private final PositionVoltage m_req = new PositionVoltage(0);

  public Intake() {
    // in init function
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // no output for error derivative

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Deploy Pos", getDeployPos());
  }

  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(-speed);
  }

  public void deployIntake(double speed) {
    m_intakeDeployMotor.set(speed);
  }

  public double getDeployPos() {
    return m_intakeDeployMotor.getPosition().getValueAsDouble();
  }

  public void setDeployPos(double rot) {
    m_intakeDeployMotor.setPosition(rot);
  }

  public double getDeployCurrent() {
    return m_intakeDeployMotor.getStatorCurrent().getValueAsDouble();
  }

  public void goToIntakePos(double pos) {
    m_req.withPosition(pos);
    m_intakeDeployMotor.setControl(m_req);
  }
}

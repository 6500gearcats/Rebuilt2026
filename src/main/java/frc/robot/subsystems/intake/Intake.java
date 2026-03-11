// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public Intake() {
    io = new TalonFXIntakeIO();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setIntakeSpeed(double speed) {
    if (io == null) {
      return;
    }
    io.setIntakeSpeed(speed);
  }

  public void deployIntake(double speed) {
    if (io == null) {
      return;
    }
    io.setDeploySpeed(speed);
  }
}

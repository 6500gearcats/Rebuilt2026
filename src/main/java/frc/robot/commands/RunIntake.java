// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;

/**
 * Runs the intake mechanism for as long as the command is scheduled.
 *
 * <p>Simultaneously runs the roller at the configured speed and applies a small hold
 * force (0.15 duty cycle) to keep the deploy arm extended. The hold force is hardcoded
 * because the deploy arm operates open-loop against gravity — gravity is always the same,
 * so 0.15 is the right value regardless of the commanded roller speed. The roller speed,
 * however, varies by use case (e.g., slower for careful pick-up, faster for quick intake).
 */
public class RunIntake extends Command {
  private Intake m_intake;
  private double speed;

  /**
   * @param intake The {@link Intake} subsystem to control.
   * @param speed  Roller motor duty cycle in [0, 1]. Positive pulls notes inward.
   *               Typical value: 0.8 for fast intake, 0.4 for gentler collection.
   */
  public RunIntake(Intake intake, double speed) {
    m_intake = intake;
    this.speed = speed;
    addRequirements(m_intake);
  }

  @Override
  public void initialize() {
  }

  /**
   * Spins the roller at {@code speed} and applies 0.15 duty cycle to hold the deploy arm down.
   * The 0.15 is calibrated to resist gravity with the current arm geometry; do not change
   * it without re-testing the arm hold position.
   */
  @Override
  public void execute() {
    m_intake.setIntakeSpeed(speed);
    m_intake.deployIntake(0.15);
  }

  /** Stops both motors and releases the deploy arm when the command ends or is interrupted. */
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeSpeed(0);
    m_intake.deployIntake(0);
  }

  /** This command runs until cancelled (e.g., by releasing the operator button). */
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that reserves the flywheel while its periodic method tracks the shot solution.
 */
public class ShootFuel extends Command {
  /** Creates a new ShootFuel. */
  private final Flywheel m_Flywheel;

  /**
   * Creates a new ShootFuel command.
   *
   * @param flywheel      flywheel subsystem
   */
  public ShootFuel(Flywheel flywheel) {
    m_Flywheel = flywheel;
    addRequirements(m_Flywheel);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Flywheel.periodic() owns the shot setpoint; no execute override is needed.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.setSpeed(60);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

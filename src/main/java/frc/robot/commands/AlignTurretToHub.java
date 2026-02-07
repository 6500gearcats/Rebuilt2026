// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTurretToHub extends Command {
  /** Creates a new AlignTurretToHub. */
  private Turret m_turret;

  public AlignTurretToHub(Turret turret) {
    m_turret = turret;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   //TODO: Fix turret alignment and angle measurment, currently jumping from ~-40 to 40 or vice versa
    double rate = m_turret.getAlignRate();
    m_turret.setSpeed(rate);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // boolean isDone = m_turret.getAlignRate() < 0.1;
    return false;
  }
}

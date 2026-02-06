// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTurretToHub extends Command {
  /** Creates a new AlignTurretToHub. */
  private Turret m_turret;
  // TODO: bounds are 0 to 0.55
  private static double lowerBound = 0.0; // TODO: These bounds are arbitrary numbers, not real positions. Update them ASAP.
  private static double upperBound = 180.0;

  public AlignTurretToHub(Turret turret) {
    m_turret = turret;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double rate = m_turret.getAlignRate();
    double pos = m_turret.getMotorPosition();

    // check if turning will cause the turret to hit the boundaries
    if(!((pos + rate <= lowerBound) || (pos + rate >= upperBound))) {
      m_turret.setSpeed(rate);
    } else {
      m_turret.setSpeed(0);
    }
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

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.FieldZone;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Flywheel;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunHopper extends Command {
  /** Creates a new RunHopper. */
  private Hopper m_hopper;
  private int counter;
  private Flywheel m_Flywheel;
  private RobotStateMachine stateMachine = RobotStateMachine.getInstance();

  public RunHopper(Hopper hopper, Flywheel flywheel) {
    m_Flywheel = flywheel;
    m_hopper = hopper;
    addRequirements(m_hopper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    counter = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((!stateMachine.isActive()) && (stateMachine.checkZone() == FieldZone.ALLIANCE)) {
      return;
    }
    if (counter > 3) {
      //if (m_Flywheel.isUpToSpeed()) {
        m_hopper.startAllMotors(-0.9, 1);
      //}
    }
    counter++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_hopper.stopAllMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

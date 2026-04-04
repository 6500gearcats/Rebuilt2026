// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.FieldZone;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.utility.RangeFinder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class UpToSpeedHopperShoot extends Command {
  /** Creates a new JasonsShooting. */
  Hopper m_Hopper;
  Flywheel m_Flywheel;
  private RobotStateMachine stateMachine = RobotStateMachine.getInstance();

  public UpToSpeedHopperShoot(Hopper m_Hopper, Flywheel m_Flywheel) {
    this.m_Hopper = m_Hopper;
    this.m_Flywheel = m_Flywheel;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((!stateMachine.isActive()) && (stateMachine.checkZone() == FieldZone.ALLIANCE)) {
      return;
    }
    if(SmartDashboard.getBoolean("Aligned", true)) {
      // m_Flywheel.setSpeed(RangeFinder.getShotVelocity(
      //   stateMachine.getTurretPose().getTranslation().getDistance(stateMachine.getTargetPose().getTranslation())));
       m_Flywheel.setSpeed(SmartDashboard.getNumber("Shoot Speed", 0));

      // if (m_Flywheel.isUpToSpeed()) {
        m_Hopper.startAllMotors(-1, 1);
      // }
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Hopper.stopAllMotors();
    m_Flywheel.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

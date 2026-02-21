// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.utility.RangeFinder;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that runs the flywheel at a supplied speed.
 */
public class ShootFuel extends Command {
  /** Creates a new ShootFuel. */
  Flywheel m_Flywheel;
  DoubleSupplier supplier;
  RobotStateMachine stateMachine = RobotStateMachine.getInstance();

  /**
   * Creates a new ShootFuel command.
   *
   * @param flywheel      flywheel subsystem
   * @param speedSupplier speed command supplier
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

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Flywheel.setSpeed(RangeFinder.getShotRPM(
        stateMachine.getTurretPose().getTranslation().getDistance(Constants.TurretConstants.HubPose.getTranslation())));
    // m_Flywheel.setSpeed(SmartDashboard.getNumber("Shoot Speed", 0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_Flywheel.setSpeed(0.7);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

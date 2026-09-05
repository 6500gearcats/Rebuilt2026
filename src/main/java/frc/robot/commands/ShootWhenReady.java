package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Waits until shoot-ready conditions are met (shooter at speed, turret tracked), then feeds.
 */
public class ShootWhenReady {
  public Command build(Hopper hopper, RobotStateMachine rsm) {
    return Commands.sequence(
        Commands.waitUntil(rsm::isShootReady),
        hopper.runOnce(() -> hopper.startAllMotors(-0.9, 1)));
  }
}

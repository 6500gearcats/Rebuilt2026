package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.superstructure.StateManager;

/**
 * Runs turret tracking and shooter spin-up in parallel until interrupted.
 */
public class AimPrep {
  public Command build(Turret turret, Shooter shooter, StateManager state) {
    return Commands.parallel(
        turret.track(state),
        shooter.shoot(state::aimParams));
  }
}

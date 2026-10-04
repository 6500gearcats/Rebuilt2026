package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.turret.Flywheel;

/** Compatibility command: the state machine owns the shared flywheel/hood setpoints. */
public class ShootFuel extends Command {
  public ShootFuel(Flywheel flywheel) { addRequirements(flywheel); }

  @Override
  public boolean isFinished() { return false; }
}

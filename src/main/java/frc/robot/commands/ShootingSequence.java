package frc.robot.commands;

import frc.robot.RobotStateMachine;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.subsystems.turret.Turret;

/** Legacy autonomous command name; uses the same readiness gates as teleop shooting. */
public class ShootingSequence extends ShootingSequenceUTS {
  public ShootingSequence(Hopper hopper, Flywheel flywheel, Turret turret) {
    super(hopper, flywheel, turret, RobotStateMachine.getInstance());
  }

  public ShootingSequence(Hopper hopper, Flywheel flywheel) {
    super(hopper, flywheel);
  }
}

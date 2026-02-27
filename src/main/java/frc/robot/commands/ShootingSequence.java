// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.utility.RangeFinder;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootingSequence extends ParallelCommandGroup {
  /** Creates a new ShootingSequence. */
  public ShootingSequence(Hopper hopper, Flywheel flywheel, Turret turret) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunHopper(hopper), new ShootFuel(flywheel), new AlignTurretToHub(turret), new CoolSnurbo(flywheel));
  }
}

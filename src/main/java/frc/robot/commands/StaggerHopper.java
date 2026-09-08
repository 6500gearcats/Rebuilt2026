// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.Hopper;

public class StaggerHopper extends SequentialCommandGroup {
  public StaggerHopper(Hopper hopper) {
    addCommands(new RunHopper(hopper).withTimeout(0.35), new WaitCommand(0.2));

  }
}

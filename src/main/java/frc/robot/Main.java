// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * Robot program entry point. Launches the WPILib robot framework with {@link Robot} as the
 * implementation class. Do not modify this class — all robot logic belongs in {@link Robot},
 * subsystems, and commands.
 */
public final class Main {
  private Main() {
  }

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}

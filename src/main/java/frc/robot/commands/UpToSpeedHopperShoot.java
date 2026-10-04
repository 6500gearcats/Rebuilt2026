// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see the WPILib BSD license file in this project.
package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.turret.Flywheel;

/** Feeds only while the shared shot and every physical readiness gate remain valid. */
public class UpToSpeedHopperShoot extends Command {
  private final BooleanSupplier canFeed;
  private final Runnable startFeed;
  private final Runnable stopFeed;
  private final Runnable recordFeedStart;
  private boolean feeding;

  public UpToSpeedHopperShoot(Hopper hopper, Flywheel flywheel) {
    this(RobotStateMachine.getInstance()::canFeedShot,
        () -> hopper.startAllMotors(-1, 1), hopper::stopAllMotors,
        () -> RobotStateMachine.getInstance().getShotCalibration().recordFeedStart());
    addRequirements(hopper);
  }

  // Package-visible callback seam tests shutdown without constructing robot hardware.
  UpToSpeedHopperShoot(BooleanSupplier canFeed, Runnable startFeed, Runnable stopFeed, Runnable recordFeedStart) {
    this.canFeed = canFeed;
    this.startFeed = startFeed;
    this.stopFeed = stopFeed;
    this.recordFeedStart = recordFeedStart;
  }

  @Override
  public void initialize() {
    feeding = false;
    stopFeed.run();
  }

  @Override
  public void execute() {
    if (canFeed.getAsBoolean()) {
      if (!feeding) { recordFeedStart.run(); }
      feeding = true;
      startFeed.run();
    } else {
      feeding = false;
      stopFeed.run();
    }
  }

  @Override
  public void end(boolean interrupted) {
    feeding = false;
    stopFeed.run();
  }

  @Override
  public boolean isFinished() { return false; }
}

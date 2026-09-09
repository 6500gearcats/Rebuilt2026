// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;


/**
 * Rumbles a controller's left motor for as long as the command is scheduled, then stops
 * both motors when the command ends.
 *
 * <p>Used as haptic feedback for game-state events (e.g., "shooter is up to speed").
 * Bind this as a {@code whileTrue} trigger so it automatically cancels when the
 * condition clears.
 *
 * <p><b>Asymmetry note:</b> {@link #execute()} sets only the left rumble motor to full;
 * {@link #end(boolean)} zeroes <em>both</em> left and right. This is intentional — on some
 * controllers a previous command or button binding may have set the right motor. Zeroing
 * both in {@code end()} guarantees the controller is quiet regardless of prior state.
 *
 * <p>TODO(unused 2026-09-09): no call sites anywhere in the codebase. The current gunner
 * left-trigger rumble binding in {@code RobotContainer.configureBindings()} sets rumble
 * directly via a {@code RunCommand} rather than using this class. Kept per
 * {@code plans/REVIEW_PROGRESS.md} decision D-3. Evaluate for deletion if still unreferenced
 * next time this is revisited.
 */
public class ControllerRumble extends Command {

  GenericHID m_controller;

  /**
   * @param controller The controller to rumble. Can be any {@link GenericHID}, including
   *                   XboxController and PS4Controller.
   */
  public ControllerRumble(GenericHID controller) {
    m_controller = controller;
  }

  @Override
  public void initialize() {}

  /** Sets the left rumble motor to full strength. */
  @Override
  public void execute() {
    m_controller.setRumble(RumbleType.kLeftRumble, 1);
  }

  /** Zeroes both rumble motors to guarantee the controller is silent after the command ends. */
  @Override
  public void end(boolean interrupted) {
    m_controller.setRumble(RumbleType.kBothRumble, 0);
  }

  /** This command runs until cancelled externally (e.g., by a trigger going false). */
  @Override
  public boolean isFinished() {
    return false;
  }
}

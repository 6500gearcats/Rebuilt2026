// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Runs the hopper for 0.35 s, then pauses for 0.2 s, then ends.
 *
 * <p>The "stagger" pattern creates a brief gap between consecutive note feeds. This gives the
 * shooter flywheel time to recover speed after the first note passes through before the next
 * note is loaded. Without the pause, back-to-back shots cause the second note to hit a
 * partially-decelerated flywheel, resulting in short or inaccurate shots.
 *
 * <p>Typical usage: called repeatedly by a while-true trigger on the gunner controller to
 * shoot multiple notes in sequence during teleop.
 *
 * <p>TODO(unused 2026-09-09): no call sites anywhere in the codebase — not bound to any
 * button, not referenced by any command group. Kept per {@code plans/REVIEW_PROGRESS.md}
 * decision D-3 in case it's staged for a teleop binding not yet written. Evaluate for
 * deletion if still unreferenced next time this is revisited. Only reachable via
 * {@link RunHopper}, which is itself unreferenced elsewhere — the two should be adopted or
 * dropped together.
 */
public class StaggerHopper extends SequentialCommandGroup {
  /**
   * @param hopper The {@link Hopper} subsystem to feed.
   */
  public StaggerHopper(Hopper hopper) {
    addCommands(new RunHopper(hopper).withTimeout(0.35), new WaitCommand(0.2));
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.RobotStateMachine.FieldZone;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Runs the hopper motors to feed notes toward the shooter.
 *
 * <p>This command is used in two contexts:
 * <ul>
 *   <li>Directly from a driver button press to feed a single note.
 *   <li>As the first step inside {@link StaggerHopper} (0.35 s on, 0.2 s pause) to create a
 *       timed gap between sequential note feeds so the shooter can recover speed between shots.
 * </ul>
 *
 * <p>Two silent behaviors that affect when the hopper actually runs:
 * <ol>
 *   <li><b>4-loop settle delay:</b> The hopper does not output in the first 4 loop cycles
 *       (~80 ms) after scheduling. This gives the shooter flywheel time to begin its spin-up
 *       before the note reaches it, preventing a stall when the note hits a still flywheel.
 *   <li><b>State/zone guard:</b> If the robot is INACTIVE <em>and</em> in its own alliance
 *       zone, the hopper skips all output. This prevents accidental in-field feeding outside
 *       of a scoring window. The guard does NOT apply in the neutral zone or opponent zone,
 *       because the robot may be collecting game pieces there regardless of scoring state.
 * </ol>
 */
public class RunHopper extends Command {
  private Hopper m_hopper;
  /** Loop-cycle counter used to enforce the 4-loop (~80 ms) settle delay. */
  private int counter;
  private RobotStateMachine stateMachine = RobotStateMachine.getInstance();

  /**
   * @param hopper The {@link Hopper} subsystem to control. Declared as a requirement so the
   *               scheduler prevents other commands from using the hopper simultaneously.
   */
  public RunHopper(Hopper hopper) {
    m_hopper = hopper;
    addRequirements(m_hopper);
  }

  /** Resets the settle counter so the delay restarts each time the command is scheduled. */
  @Override
  public void initialize() {
    counter = 0;
  }

  /**
   * Runs the hopper motors after the 4-loop settle delay, unless the scoring guard blocks output.
   *
   * <p>Motor speeds: belt motor at −0.9 (full speed toward shooter), cross-feed at 1.0.
   */
  @Override
  public void execute() {
    // Guard: if the robot is inactive AND inside its own alliance zone, skip output.
    // In neutral/opponent zones the robot collects notes regardless of scoring windows.
    if ((!stateMachine.isActive()) && (stateMachine.checkZone() == FieldZone.ALLIANCE)) { return; }
    if (counter > 3) {
      m_hopper.startAllMotors(-0.9, 1);
    }
    counter++;
  }

  /** Stops all hopper motors whether the command ended normally or was interrupted. */
  @Override
  public void end(boolean interrupted) {
    m_hopper.stopAllMotors();
  }

  /** This command runs until cancelled or interrupted externally (e.g., by a timeout). */
  @Override
  public boolean isFinished() {
    return false;
  }
}

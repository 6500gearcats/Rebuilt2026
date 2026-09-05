package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.superstructure.StateManager;

/**
 * Runs turret tracking and shooter spin-up simultaneously until interrupted.
 *
 * <h2>What This Does</h2>
 * Pressing the gunner's left trigger starts two things in parallel:
 * <ol>
 *   <li><b>Turret tracking</b> — {@link Turret#track(StateManager)} continuously solves for the
 *       required turret angle each loop cycle and drives the mechanism toward it. Tracking stays
 *       "coarse" (larger position tolerance) until the shooter is also at speed, then tightens.
 *   <li><b>Shooter spin-up</b> — {@link Shooter#shoot(java.util.function.Supplier)} asks the
 *       aim pipeline for the required hood angle and wheel speed each loop and drives both.
 *       The shooter keeps running if interrupted (see Shooter.shoot javadoc) to maintain wheel
 *       speed for a follow-up shot.
 * </ol>
 * The command runs indefinitely — the scheduler interrupts it when the trigger is released or
 * when a higher-priority command needs the turret or shooter.
 *
 * <h2>Why Parallel, Not Sequence?</h2>
 * Turret slew and shooter spin-up are independent mechanisms that both need to be ready before
 * the shot is fired. Running them in sequence (turret first, then shooter) would waste time —
 * the shooter might need 1–2 seconds to reach speed, and the turret might need 0.5 seconds to
 * slew. Parallel cuts the worst-case time to {@code max(turret_time, shooter_time)}.
 *
 * <h2>How to Know When to Fire</h2>
 * This command does NOT feed the ball. After pressing the aim trigger, the gunner presses the
 * left bumper to trigger {@link ShootWhenReady}, which waits until
 * {@link frc.robot.RobotStateMachine#isShootReady()} is true (both turret tracked and shooter
 * at speed) before starting the hopper.
 *
 * <h2>Subsystem Requirements</h2>
 * {@code Commands.parallel()} inherits the requirements of all sub-commands. This command
 * requires both the {@code Turret} and {@code Shooter} subsystems, preventing other commands
 * from accidentally driving either while aiming.
 */
public class AimPrep {

  /**
   * Returns the built parallel command.
   *
   * @param turret  turret subsystem (requirement: turret is dedicated to tracking while active)
   * @param shooter shooter subsystem (requirement: shooter is dedicated to spin-up while active)
   * @param state   state bridge providing robot pose and aim parameters to the turret
   */
  public Command build(Turret turret, Shooter shooter, StateManager state) {
    return Commands.parallel(
        turret.track(state),
        shooter.shoot(state::aimParams));
  }
}

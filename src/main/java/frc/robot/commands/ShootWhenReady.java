package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.hopper.Hopper;

/**
 * Waits until the robot is in a shoot-ready state, then runs the hopper to feed the ball.
 *
 * <h2>What "Shoot Ready" Means</h2>
 * {@link RobotStateMachine#isShootReady()} returns {@code true} when all of the following are
 * simultaneously true:
 * <ul>
 *   <li>The shooter wheel speed is within {@link frc.robot.aiming.AimParams#deltaOutput} of the
 *       required speed for the current distance.
 *   <li>The hood angle is within {@link frc.robot.aiming.AimParams#deltaPitch} of the required
 *       angle for the current distance.
 *   <li>The turret is within {@link frc.robot.aiming.AimParams#deltaYaw} of the target yaw and
 *       is actively tracking (not just at home).
 * </ul>
 * The condition is checked every robot loop (50 Hz) by polling the
 * {@link frc.robot.superstructure.StateManager#shootReady} trigger.
 *
 * <h2>Command Sequence</h2>
 * <pre>
 *   waitUntil(isShootReady)  → hopper.startEnd(startAllMotors, stopAllMotors)
 * </pre>
 * <ol>
 *   <li>The {@code waitUntil} step occupies zero subsystem requirements while waiting, so the
 *       turret and shooter (running under {@link AimPrep}) can continue operating in parallel
 *       on the same command group without a scheduling conflict.
 *   <li>Once ready, {@link Hopper#startEnd startEnd} starts both hopper motors and keeps them
 *       running until the command ends or is interrupted.
 * </ol>
 *
 * <h2>Why startEnd Instead of runOnce</h2>
 * {@link edu.wpi.first.wpilibj2.command.SubsystemBase#startEnd startEnd} guarantees that
 * {@link Hopper#stopAllMotors()} is called in the command's {@code end()} method, regardless
 * of whether the command ends normally or is interrupted. This is critical in autonomous:
 * when a timed auto step expires, the scheduler interrupts the command and the hopper stops
 * cleanly. With {@code runOnce}, the motors would start and then keep running into the next
 * auto step because there is no cleanup.
 *
 * <h2>Interaction with AimPrep</h2>
 * In teleop, this command is bound to the gunner's left bumper alongside {@link AimPrep} on the
 * left trigger. The typical flow is:
 * <ol>
 *   <li>Hold left trigger → AimPrep runs (turret tracks, shooter spins up).
 *   <li>Press left bumper → ShootWhenReady waits for ready condition, then feeds.
 *   <li>Release left bumper → hopper stops immediately via {@code stopAllMotors}.
 *   <li>Release left trigger → AimPrep ends, turret stops tracking, shooter stops.
 * </ol>
 *
 * <h2>In Autonomous</h2>
 * Named commands like {@code "ShootFuel3s"} wrap a parallel of {@link AimPrep} and this command
 * inside a {@code .withTimeout(3.0)}. When the timeout fires, both commands are interrupted, the
 * hopper stops, and the path planner continues to the next segment.
 */
public class ShootWhenReady {

  /**
   * Returns the built sequence command.
   *
   * @param hopper the hopper subsystem; required by the {@code startEnd} portion so the scheduler
   *               knows to interrupt hopper operation when this command ends
   * @param rsm    the robot state machine providing the {@code isShootReady} condition
   */
  public Command build(Hopper hopper, RobotStateMachine rsm) {
    return Commands.sequence(
        // Phase 1: wait — no subsystem requirement, so AimPrep keeps running in parallel.
        Commands.waitUntil(rsm::isShootReady),
        // Phase 2: feed — startEnd means stopAllMotors() is called on any end, including interrupts.
        hopper.startEnd(() -> hopper.startAllMotors(-0.9, 1), hopper::stopAllMotors));
  }
}

package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotStateMachine;
import frc.robot.aiming.AimParams;

/**
 * Bridge between the Hackbots {@code Turret} API and the Gearcats {@link RobotStateMachine}
 * singleton.
 *
 * <h2>Why This Class Exists</h2>
 * The Hackbots {@code Turret.java} (copied from the Hackbots 2026 codebase) was written to call
 * methods on a {@code StateManager} object — specifically {@link #robotPose()},
 * {@link #aimParams()}, and the {@link #shootReady} trigger. The Hackbots codebase has a full
 * {@code Superstructure} class hierarchy that owns all of this state, but integrating that whole
 * architecture would require rewriting large parts of the Gearcats codebase.
 *
 * <p>Instead, this thin adapter delegates every call to {@link RobotStateMachine#getInstance()},
 * which already manages pose, aiming, and readiness for the Gearcats robot. {@code Turret} gets
 * the interface it expects; we keep our existing singleton structure unchanged.
 *
 * <h2>Design Pattern</h2>
 * This is a classic <em>Adapter</em> (or "wrapper") pattern:
 * <ul>
 *   <li>The {@code Turret} (client) expects a {@code StateManager} with specific methods.
 *   <li>The {@code RobotStateMachine} (adaptee) already has the data but under different method
 *       names and as a singleton rather than a passed-in reference.
 *   <li>This class is the adapter — it bridges the API mismatch.
 * </ul>
 *
 * <h2>Lifecycle</h2>
 * One instance is created in {@link frc.robot.RobotContainer} and passed to both
 * {@link frc.robot.commands.AimPrep} (via {@code Turret.track(state)}) and directly to
 * {@code Turret.track(state)}. The same instance is safe to reuse because all methods delegate
 * to the thread-safe (single-threaded robot loop) {@link RobotStateMachine}.
 */
public class StateManager {

  private final RobotStateMachine rsm;

  /**
   * A WPILib {@link Trigger} that is {@code true} when the robot is ready to fire —
   * meaning the shooter is at speed, the hood is at the correct angle, and the turret is
   * tracking the target within tolerance.
   *
   * <p>This is a public field (not a method) so it can be bound directly to other Triggers and
   * Command factories using {@code Trigger.and()}, {@code Trigger.onTrue()}, etc. without
   * allocating a new lambda each call.
   *
   * <p>Polled every robot loop by the WPILib CommandScheduler.
   */
  public final Trigger shootReady;

  /**
   * Creates a {@code StateManager} backed by the given {@link RobotStateMachine}.
   *
   * @param rsm the robot state machine singleton; in normal usage this is always
   *            {@link RobotStateMachine#getInstance()}
   */
  public StateManager(RobotStateMachine rsm) {
    this.rsm = rsm;
    // Wrap the boolean-returning method in a Trigger so the scheduler can poll it efficiently.
    this.shootReady = new Trigger(rsm::isShootReady);
  }

  /**
   * Returns the current best estimate of the robot's field-relative pose (position + heading).
   *
   * <p>Internally this calls {@link RobotStateMachine#getPose()}, which fuses wheel odometry with
   * PhotonVision camera estimates filtered by distance from the odometry estimate.
   *
   * <p>Used by {@code Turret.track()} to convert the field-frame aim yaw into a robot-relative
   * turret mechanism angle: {@code mechanismAngle = aimYaw - robotHeading + kForwards}.
   */
  public Pose2d robotPose() {
    return rsm.getPose();
  }

  /**
   * Returns the current lead-compensated shot parameters (yaw, pitch, speed, TOF, status).
   *
   * <p>Internally calls {@link RobotStateMachine#getAimParams()}, which runs the full pipeline:
   * <ol>
   *   <li>{@link frc.robot.aiming.LeadCompensator#computeLeadTarget LeadCompensator} shifts the
   *       hub position to account for robot velocity during ball flight.
   *   <li>{@link frc.robot.aiming.ToFAim#update ToFAim} looks up pitch, speed, and TOF for the
   *       resulting virtual target distance.
   * </ol>
   *
   * <p>Returns {@link AimParams#impossible()} when no AprilTag pose is available or when no
   * feasible trajectory exists within the hood angle and speed constraints.
   */
  public AimParams aimParams() {
    return rsm.getAimParams();
  }
}

package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Computes a lead-compensated virtual target position so the shooter accounts for robot motion
 * during ball flight.
 *
 * <h2>The Problem</h2>
 * A ball fired from a moving robot takes roughly 0.3–0.8 seconds to reach the hub. During that
 * time the robot keeps moving. If we aim directly at the hub from our current position, the ball
 * will miss because the shooter has moved by the time the ball arrives.
 *
 * <h2>The Solution: Virtual Target</h2>
 * Instead of aiming at the real hub, we aim at a <em>virtual target</em> — a point shifted
 * backward by {@code fieldVelocity × tof}. When the ball is launched toward this virtual target
 * the ball's path in field coordinates still passes through the real hub at arrival time,
 * because the robot's motion adds to the ball's field-frame velocity.
 *
 * <p>Mathematically, the required field-frame launch angle is:
 * <pre>
 *   θ = atan2(hub - (robotPos + velocity × tof))
 *     = atan2(virtualTarget - robotPos)
 * </pre>
 * where {@code virtualTarget = hub - velocity × tof}.
 *
 * <h2>Why Iterate?</h2>
 * The {@code tof} depends on distance to the virtual target (looked up from measured data), but
 * the virtual target position depends on {@code tof}. We iterate to find the fixed point:
 * <ol>
 *   <li>Use the current best virtual target to ask the inner aim strategy for {@code tof}.
 *   <li>Shift the real hub by {@code -velocity × tof} to get a new virtual target.
 *   <li>Repeat until the virtual target stops moving ({'@literal <}1 mm change).
 * </ol>
 * In practice, convergence takes 3–4 iterations even at top FRC drive speeds (~5 m/s).
 *
 * <h2>Why kZero Velocity to the Inner Strategy?</h2>
 * The inner strategy ({@link ToFAim}) also has its own internal lead-compensation loop that
 * shifts the shooter position by {@code velocity × tof}. If we passed the real velocity here,
 * lead would be applied twice — once by this class (virtual target shift) and once by ToFAim
 * internally. Passing {@link Translation2d#kZero} tells ToFAim "treat the robot as stationary"
 * so it only solves for pitch/speed/tof from the static virtual target, and we own the lead.
 *
 * <h2>Relationship to ToFAim</h2>
 * Both approaches are mathematically equivalent and produce the same final yaw angle.
 * The advantage of this class is that it stores {@link #computeLeadTarget the converged virtual
 * target} as an intermediate value, making the lead offset observable for telemetry
 * ({@code Aiming/LeadOffsetXM}, {@code Aiming/LeadOffsetYM} in SmartDashboard).
 *
 * @see ToFAim
 * @see RobotStateMachine#getAimParams()
 */
public class LeadCompensator {

  /**
   * Returns the converged virtual target pose that accounts for the robot's field velocity.
   *
   * <p>The returned pose is the hub position shifted by {@code -fieldVelocity × tof} at
   * convergence. When the robot is stationary, this equals {@code hubPose} exactly. When moving,
   * it shifts "behind" the hub in the direction of travel by roughly
   * {@code speed × tof} meters (typically 0.3–1.5 m at competition speeds).
   *
   * <p>After calling this method, pass the returned pose to the inner aim strategy with
   * {@link Translation2d#kZero} velocity to get final pitch/speed/yaw without double-counting
   * the lead:
   * <pre>
   *   Pose3d lead = LeadCompensator.computeLeadTarget(hub, shooter, vel, m_tofAim);
   *   AimParams params = m_tofAim.update(lead, shooter, Translation2d.kZero);
   * </pre>
   *
   * @param hubPose      true hub position on the field (from the AprilTag layout)
   * @param shooterPose  current shooter pose in field coordinates
   * @param fieldVelocity robot field-relative velocity in m/s; typically from
   *                     {@code ChassisSpeeds.fromRobotRelativeSpeeds(...)}
   * @param strategy     inner aim strategy — called with {@link Translation2d#kZero} velocity
   *                     so it computes pitch/speed/tof for a static virtual target
   * @return converged virtual target; equals {@code hubPose} when stationary or when the
   *         inner strategy returns {@link AimParams.AimStatus#Impossible Impossible}
   */
  public static Pose3d computeLeadTarget(
      Pose3d hubPose,
      Pose3d shooterPose,
      Translation2d fieldVelocity,
      AimStrategy strategy) {

    // Start with the real hub; we'll shift it each iteration as we refine our tof estimate.
    Pose3d best = hubPose;

    for (int i = 0; i < 5; i++) {
      // Ask the inner strategy: if the target were at 'best' and the robot were stationary,
      // what would be the pitch, speed, and time-of-flight?
      // kZero velocity → no internal lead shift inside ToFAim.
      AimParams params = strategy.update(best, shooterPose, Translation2d.kZero);

      // If the inner strategy cannot solve (e.g., out of range), stop iterating and return
      // whatever we have so far — the calling code will see AimStatus.Impossible from the
      // subsequent m_tofAim.update() call.
      if (!params.isOk() || params.tof <= 0) break;

      // Compute the new virtual target:
      //   virtualTarget = realHub - velocity × tof
      // The Z and rotation of the hub are preserved because we only lead-compensate the
      // horizontal plane (the ball's flight time in the vertical axis is handled by pitch).
      Pose3d next = new Pose3d(
          hubPose.getX() - fieldVelocity.getX() * params.tof,
          hubPose.getY() - fieldVelocity.getY() * params.tof,
          hubPose.getZ(),
          Rotation3d.kZero);

      // Convergence check: if the virtual target barely moved from the last iteration
      // (< 1 mm), we've found the fixed point. Accept and stop early.
      if (next.getTranslation().getDistance(best.getTranslation()) < 1e-3) {
        best = next;
        break;
      }
      best = next;
    }

    return best;
  }
}

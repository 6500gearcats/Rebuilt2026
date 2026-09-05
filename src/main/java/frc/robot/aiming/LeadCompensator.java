package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Computes a lead-compensated virtual target position for a moving robot.
 *
 * <p>Iterates up to 5 times: each pass uses the time-of-flight from the inner aim strategy to shift
 * the hub position backward by {@code fieldVelocity * tof}, converging to the correct virtual aim
 * point. The inner strategy is called with zero velocity so lead is not double-counted.
 *
 * <p>Mathematically equivalent to passing fieldVelocity directly to {@link ToFAim}, but separates
 * the lead-offset computation so it can be logged independently.
 */
public class LeadCompensator {

  /**
   * Returns the converged virtual target pose that accounts for the robot's field velocity.
   *
   * @param hubPose      true hub position on the field
   * @param shooterPose  current shooter pose
   * @param fieldVelocity robot field-relative velocity (m/s)
   * @param strategy     inner aim strategy (called with zero velocity to avoid double-counting)
   * @return virtual target pose; equals hubPose when stationary or when the inner strategy returns
   *         Impossible on the first iteration
   */
  public static Pose3d computeLeadTarget(
      Pose3d hubPose,
      Pose3d shooterPose,
      Translation2d fieldVelocity,
      AimStrategy strategy) {

    Pose3d best = hubPose;

    for (int i = 0; i < 5; i++) {
      AimParams params = strategy.update(best, shooterPose, Translation2d.kZero);
      if (!params.isOk() || params.tof <= 0) break;

      Pose3d next = new Pose3d(
          hubPose.getX() - fieldVelocity.getX() * params.tof,
          hubPose.getY() - fieldVelocity.getY() * params.tof,
          hubPose.getZ(),
          Rotation3d.kZero);

      if (next.getTranslation().getDistance(best.getTranslation()) < 1e-3) {
        best = next;
        break;
      }
      best = next;
    }

    return best;
  }
}

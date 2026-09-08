package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Strategy interface for calculating shot parameters from the current robot state.
 *
 * <p>Implementations represent distinct aiming modes (e.g., physics-based ballistic
 * calculation via {@link PhysicsAim}, calibration-table lookup via {@link ToFAim},
 * manual SmartDashboard tuning via {@link TuneAim}). The active strategy is selected
 * in {@link frc.robot.RobotStateMachine} and swapped at runtime without changing the
 * calling code.
 *
 * <p>All implementations must return an {@link AimParams} with a {@code Possible} status
 * when a valid solution exists, or {@code Impossible} when the geometry cannot be solved
 * (e.g., target behind the robot, out of hood range).
 */
public interface AimStrategy {
  /**
   * Computes the shot parameters needed to hit {@code target} from {@code shooter}.
   *
   * @param target   The 3D pose of the scoring target (hub center) in field coordinates.
   * @param shooter  The 3D pose of the shooter mechanism in field coordinates.
   * @param velocity The robot's current field-relative velocity (x, y in m/s),
   *                 used for lead compensation when the robot is moving.
   * @return The computed {@link AimParams}, with status {@code Possible} on success
   *         or {@code Impossible} if no valid shot exists.
   */
  public AimParams update(Pose3d target, Pose3d shooter, Translation2d velocity);
}

package frc.robot.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.aiming.AimParams.AimStatus;
import frc.robot.aiming.AimParams.SpeedControl;

/**
 * Ballistic trajectory solver that computes shooter angle and speed from first-principles
 * kinematics, taking robot motion into account.
 *
 * <p>Unlike {@link ToFAim}, which fits a polynomial to empirical calibration data,
 * {@code PhysicsAim} derives launch parameters analytically from the 3D offset between
 * the shooter and the target.
 *
 * <h2>Algorithm overview</h2>
 * <ol>
 *   <li>The ball must arrive at the target with a specific downward velocity. The range of
 *       acceptable descent speeds ({@link #minDescentVelocity} to {@link #maxDescentVelocity})
 *       defines the family of valid trajectories — a shallow arc at min speed, a steep arc at
 *       max speed.</li>
 *   <li>{@link #quicksolve} analytically finds launch velocity for any given descent speed
 *       using a quadratic equation derived from projectile-motion equations.</li>
 *   <li>{@link #update} first tries the minimum (lowest-energy) trajectory. If that satisfies
 *       {@link AimConstraints}, it returns immediately. Otherwise it binary-searches the
 *       descent-speed range across {@link #ITERATIONS} iterations to find the shallowest
 *       trajectory that still satisfies the constraints.</li>
 *   <li>A final yaw sanity check rejects any solution whose computed launch direction differs
 *       from the actual direction to the target by more than 0.8 π radians (144°). Such
 *       solutions are mathematically valid but physically impossible — the shooter cannot
 *       reach the target in that direction.</li>
 * </ol>
 */
public class PhysicsAim implements AimStrategy {
  /** Number of binary-search refinement passes over the descent-speed interval. */
  private static final int ITERATIONS = 5;

  private final AimConstraints constraints;

  /**
   * Minimum acceptable terminal descent speed (m/s, positive = downward).
   * Corresponds to the flattest, lowest-energy arc. Use a value calibrated so the
   * ball clears the hub lip at the nearest shot distance.
   */
  private final double minDescentVelocity;

  /**
   * Maximum acceptable terminal descent speed (m/s, positive = downward).
   * Corresponds to the steepest arc. Higher values allow closer shots but reduce
   * scoring consistency on the rim.
   */
  private final double maxDescentVelocity;

  /**
   * @param constraints        Physical limits on pitch angle and projectile speed.
   * @param minDescentVelocity Minimum terminal descent speed in m/s — flattest acceptable arc.
   * @param maxDescentVelocity Maximum terminal descent speed in m/s — steepest acceptable arc.
   */
  public PhysicsAim(AimConstraints constraints, double minDescentVelocity,
      double maxDescentVelocity) {
    this.constraints = constraints;
    this.minDescentVelocity = minDescentVelocity;
    this.maxDescentVelocity = maxDescentVelocity;
  }

  /**
   * Computes the optimal launch parameters to reach {@code target} from {@code shooter}.
   *
   * <p>The method first attempts the minimum-descent-speed (flattest) trajectory, which
   * requires the least projectile speed. If the flat trajectory violates
   * {@link AimConstraints}, it binary-searches the descent-speed range to find the
   * shallowest feasible arc.
   *
   * <p>Returns {@link AimParams#impossible()} when no trajectory within the descent-speed
   * range can satisfy the constraints, or when the computed yaw deviates more than 144° from
   * the straight-line direction to the target.
   *
   * @param target          3D pose of the scoring target (hub opening center).
   * @param shooter         3D pose of the shooter exit point.
   * @param shooterVelocity Field-relative robot velocity at the shooter (m/s in x and y).
   *                        Subtracted from projectile velocity so the ball travels to the
   *                        target regardless of robot motion.
   * @return Launch parameters ({@link AimParams}), or {@link AimParams#impossible()} if
   *         no feasible solution exists.
   */
  public AimParams update(Pose3d target, Pose3d shooter, Translation2d shooterVelocity) {
    Translation3d offset = target.getTranslation().minus(shooter.getTranslation());

    AimParams minParams = quicksolve(offset, shooterVelocity, minDescentVelocity);
    AimParams maxParams = quicksolve(offset, shooterVelocity, maxDescentVelocity);

    double minPitch = minParams.pitch.getRadians();
    double maxPitch = maxParams.pitch.getRadians();

    boolean solutionExists = minPitch <= constraints.maxShooterAngle().getRadians()
        && maxPitch >= constraints.minShooterAngle().getRadians();

    if (!solutionExists) {
      return AimParams.impossible();
    }

    boolean minWorks = constraints.check(minParams);
    if (minWorks) {
      minParams.status = AimStatus.Possible;
      minParams.control = SpeedControl.ProjectileVelocity;
      return minParams;
    }

    double lower = minDescentVelocity;
    double upper = maxDescentVelocity;

    AimParams best = AimParams.impossible();

    for (int i = 0; i < ITERATIONS; i++) {
      double guess = 0.5 * (lower + upper);
      AimParams output = quicksolve(offset, shooterVelocity, guess);
      boolean ok = constraints.check(output);
      if (ok) {
        upper = guess;
        best = output;
        best.status = AimStatus.Possible;
        continue;
      }
      double pitch = output.pitch.getRadians();
      if (pitch > constraints.maxShooterAngle().getRadians()) {
        upper = guess;
        continue;
      }
      if (pitch < constraints.minShooterAngle().getRadians()) {
        lower = guess;
        continue;
      }
      if (!ok) {
        upper = guess;
      }
    }

    if (best.status == AimStatus.Impossible) {
      return AimParams.impossible();
    }

    // Sanity check: the computed yaw must be within 144° of the straight-line direction to
    // the target. Solutions outside this window are kinematically valid but physically absurd
    // (the ball would curve back toward the shooter or wrap around the field).
    Rotation2d towardsTarget = Rotation2d.fromRadians(Math.atan2(offset.getY(), offset.getX()));
    double diff = MathUtil.angleModulus(Math.abs(towardsTarget.minus(best.yaw).getRadians()));
    if (diff > 0.8 * Math.PI) {
      return AimParams.impossible();
    }

    best.control = SpeedControl.ProjectileVelocity;
    return best;
  }

  /**
   * Analytically solves for launch velocity given a fixed terminal descent speed.
   *
   * <h2>Derivation</h2>
   * <p>Let g = 9.81 m/s². The ball's vertical position at time t is:
   * <pre>  z(t) = vz₀·t − ½g·t²</pre>
   * Its vertical velocity at time t is:
   * <pre>  ż(t) = vz₀ − g·t</pre>
   * We require it to descend at {@code finalDescentSpeed} on arrival, so:
   * <pre>  ż(t) = −finalDescentSpeed  →  vz₀ = g·t − finalDescentSpeed</pre>
   * Substituting into the height equation and rearranging:
   * <pre>  ½g·t² − finalDescentSpeed·t − dz = 0</pre>
   * This is the quadratic {@code a·t² + b·t + c = 0} with
   * {@code a = ½g}, {@code b = −finalDescentSpeed}, {@code c = −dz}.
   * We take the positive root for a physically meaningful (forward-in-time) solution.
   *
   * <p>With flight time known, horizontal velocities follow from uniform motion:
   * {@code vx = dx/t}, {@code vy = dy/t}. Robot velocity is subtracted so the
   * ball travels to the target regardless of whether the robot is moving.
   *
   * @param offset            Vector from shooter to target (meters: x forward, y left, z up).
   * @param robotVelocity     Field-relative robot velocity in the horizontal plane (m/s).
   * @param finalDescentSpeed Desired downward speed (m/s, positive = downward) at impact.
   * @return {@link AimParams} containing launch speed ({@code output}), pitch, and yaw.
   *         {@code status} is left at its default; the caller sets it.
   */
  public static AimParams quicksolve(
      Translation3d offset,
      Translation2d robotVelocity,
      double finalDescentSpeed) {

    AimParams params = new AimParams();

    double dx = offset.getX();
    double dy = offset.getY();
    double dz = offset.getZ();

    // Quadratic coefficients — see Javadoc derivation above.
    double a = 0.5 * 9.81;
    double b = -finalDescentSpeed;
    double c = -dz;
    double discriminant = b * b - 4 * a * c;
    double t = (-b + Math.sqrt(discriminant)) / (2 * a);

    double vx = dx / t;
    double vy = dy / t;
    double vz = 9.81 * t - finalDescentSpeed; // initial vertical launch speed

    // Subtract robot velocity so the ball's ground-frame path points to the target.
    vx -= robotVelocity.getX();
    vy -= robotVelocity.getY();

    double v = Math.sqrt(vx * vx + vy * vy + vz * vz);
    Rotation2d yaw = Rotation2d.fromRadians(Math.atan2(vy, vx));
    Rotation2d pitch = Rotation2d.fromRadians(Math.asin(vz / v));

    params.output = v;
    params.pitch = pitch;
    params.yaw = yaw;

    return params;
  }
}

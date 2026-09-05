package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.aiming.AimParams.AimStatus;
import frc.robot.aiming.AimParams.SpeedControl;

/**
 * Aim strategy that uses experimentally measured time-of-flight data to compute the ideal shot
 * parameters for a moving robot.
 *
 * <h2>What Is "Time of Flight"?</h2>
 * When a ball is fired, it travels through the air for some duration before reaching the hub.
 * That duration — the <em>time of flight</em> (TOF) — depends on the straight-line distance from
 * the shooter to the hub and on the launch angle and speed. Longer shots take longer. The TOF for
 * this robot was measured by filming shots at known distances with a high-speed camera; those
 * measurements are stored in {@link ShooterConstants#scoringMeasurements} as an
 * {@link AimMeasurement} list and loaded into interpolating lookup tables in this class.
 *
 * <h2>The Moving-Robot Problem</h2>
 * If the robot were stationary, aiming is simple: look up the distance to the hub, use the table
 * to find the correct pitch and speed, and point the turret directly at the hub.
 *
 * <p>When the robot is moving, the ball inherits the robot's velocity in addition to the launch
 * velocity. To compensate, the turret must aim at a different field-frame angle so the ball still
 * arrives at the hub. Specifically, the required field-frame launch angle θ is:
 * <pre>
 *   θ = atan2(hub - (shooterPos + robotVelocity × tof))
 * </pre>
 * where {@code shooterPos + robotVelocity × tof} is the position the shooter will be at when the
 * ball arrives (call this {@code afterShooting}). The formula says: point the turret from the
 * future robot position toward the hub — that direction, in field coordinates, is the angle the
 * ball must leave the launcher at (adding the robot's field velocity gives the ball a path that
 * arrives at the hub).
 *
 * <h2>The Iterative Convergence Loop</h2>
 * There is a circular dependency: {@code tof} depends on distance, which depends on
 * {@code afterShooting}, which depends on {@code tof}. We break this by iterating:
 * <ol>
 *   <li>Start with {@code afterShooting = shooterPos} (zero lead estimate).
 *   <li>Compute distance from {@code afterShooting} to hub.
 *   <li>Look up {@code tof} from the distance table.
 *   <li>Compute a new {@code afterShooting = shooterPos + velocity × tof}.
 *   <li>If the new estimate changed less than {@link #EPSILON} metres, we've converged → stop.
 *   <li>Otherwise repeat up to {@link #ITERATIONS} times.
 * </ol>
 * In practice, convergence happens in 3–4 iterations at FRC drive speeds. If after 15 iterations
 * convergence has not been reached (robot is oscillating or sensor data is wildly noisy), the
 * shot is declared {@link AimStatus#Impossible Impossible}.
 *
 * <h2>Pitch and Speed</h2>
 * Once {@code afterShooting} has converged, the final distance {@code d = |afterShooting - hub|}
 * is used to look up the correct hood pitch angle and shooter wheel speed from the tables. These
 * values were measured at rest and are a good approximation even when moving because the distance
 * correction is small (≈1 m at 2 m/s for 0.5 s TOF).
 *
 * <h2>Usage</h2>
 * An instance is created once in {@code RobotStateMachine} and called every control loop via
 * {@link frc.robot.aiming.LeadCompensator LeadCompensator}. Call it via
 * {@link #update(Pose3d, Pose3d, Translation2d)} each loop.
 *
 * @see LeadCompensator
 * @see AimMeasurement
 * @see ShooterConstants#scoringMeasurements
 */
public class ToFAim implements AimStrategy {

  /** Convergence threshold: stop iterating when the shooter-position estimate moves less than
   *  this many metres between iterations. 1 mm is well within FRC aiming tolerance. */
  static final double EPSILON = 1e-3;

  /** Maximum convergence iterations before declaring the shot impossible. 15 iterations handles
   *  every realistic FRC scenario; oscillation beyond this indicates bad sensor data. */
  static final int ITERATIONS = 15;

  private final AimConstraints constraints;

  /** Interpolating map: distance (metres) → time-of-flight (seconds). */
  private final InterpolatingDoubleTreeMap timeMap;
  /** Interpolating map: distance (metres) → hood angle (degrees). */
  private final InterpolatingDoubleTreeMap speedMap;
  /** Interpolating map: distance (metres) → shooter wheel speed (mechanism control units). */
  private final InterpolatingDoubleTreeMap pitchMap;

  /**
   * Builds the three interpolating lookup tables from the list of experimental measurements.
   *
   * @param measurements list of (distance, pitch, speed, time) data points; the more points
   *                     and the wider the range, the more accurate extrapolation at the edges
   * @param constraints  min/max hood angle and max output; shots outside these limits are marked
   *                     Impossible so the robot doesn't attempt physically unreachable trajectories
   */
  public ToFAim(List<AimMeasurement> measurements, AimConstraints constraints) {
    this.constraints = constraints;

    timeMap = new InterpolatingDoubleTreeMap();
    speedMap = new InterpolatingDoubleTreeMap();
    pitchMap = new InterpolatingDoubleTreeMap();

    for (AimMeasurement measurement : measurements) {
      double distance = measurement.distance().in(Meters);
      double pitch = measurement.pitch().getDegrees();
      double tof = measurement.time().in(Seconds);
      timeMap.put(distance, tof);
      pitchMap.put(distance, pitch);
      speedMap.put(distance, measurement.shooterControl());
    }
  }

  /**
   * Computes the aim parameters for the current robot state.
   *
   * <p>This is the main entry point called each control loop (typically via
   * {@link LeadCompensator#computeLeadTarget LeadCompensator} first, then this method with
   * {@link Translation2d#kZero} velocity so lead compensation is not double-applied).
   *
   * @param aimTarget     target position in field coordinates — either the real hub or a
   *                      lead-compensated virtual target from {@link LeadCompensator}
   * @param shooterPose   current shooter pose in field coordinates
   * @param shooterVelocity robot field-relative velocity; pass {@link Translation2d#kZero} when
   *                      lead compensation is handled externally by {@link LeadCompensator}
   * @return computed shot parameters; status is {@link AimStatus#Impossible} if the shot cannot
   *         converge or if the result violates {@code constraints}
   */
  public AimParams update(Pose3d aimTarget, Pose3d shooterPose, Translation2d shooterVelocity) {
    // Project 3D poses to 2D — the turret only rotates in the horizontal plane.
    Translation2d target = aimTarget.getTranslation().toTranslation2d();
    Translation2d start = shooterPose.getTranslation().toTranslation2d();

    // 'afterShooting' represents where the shooter will be when the ball arrives.
    // We start with the current position (zero-lead estimate) and refine each iteration.
    Translation2d afterShooting = start;

    AimStatus status = AimStatus.Impossible; // Assume failure; set Possible on convergence.

    // 'distance' is updated each iteration as afterShooting evolves.
    double distance = start.minus(target).getNorm();
    double tof;

    // --- Convergence loop -----------------------------------------------
    // Each iteration refines the estimate of where the robot will be (afterShooting) by:
    //   1. Measuring distance from the current afterShooting estimate to the target.
    //   2. Looking up TOF for that distance.
    //   3. Computing where the robot actually will be after that TOF.
    //   4. Checking whether the estimate changed; if negligible, we've converged.
    for (int i = 0; i < ITERATIONS; i++) {
      distance = afterShooting.minus(target).getNorm();
      tof = timeMap.get(distance);

      // Where will the shooter be tof seconds from now?
      Translation2d newAfterShooting = start.plus(shooterVelocity.times(tof));

      double error = newAfterShooting.minus(afterShooting).getNorm();
      if (error < EPSILON) {
        // The estimate has settled — the self-consistent solution has been found.
        status = AimStatus.Possible;
        break;
      }
      afterShooting = newAfterShooting;
    }

    if (status == AimStatus.Impossible) {
      // Didn't converge in ITERATIONS steps — sensor data is unreliable or robot is
      // oscillating wildly. Return Impossible so the robot does not attempt the shot.
      return AimParams.impossible();
    }

    // --- Final shot parameters from the converged distance -----------------
    double pitch = pitchMap.get(distance);
    double shooterControl = speedMap.get(distance);

    AimParams params = new AimParams();
    params.pitch = Rotation2d.fromDegrees(pitch);
    params.output = shooterControl;
    params.control = SpeedControl.MechanismControl;
    params.tof = timeMap.get(distance); // Store TOF so LeadCompensator and telemetry can use it.

    // The required FIELD-FRAME launch angle: the direction from the robot's future position
    // (afterShooting) to the hub. Because the ball inherits the robot's field velocity, aiming
    // in this direction from the current position sends the ball to the hub on arrival.
    // Turret.track() will subtract the robot heading to convert to a robot-relative angle.
    Translation2d finalOffset = target.minus(afterShooting);
    params.yaw = Rotation2d.fromRadians(Math.atan2(finalOffset.getY(), finalOffset.getX()));

    // Validate against physical hood and output limits. A Possible shot that exceeds constraints
    // is marked Impossible so we don't stress the mechanism or produce a weak trajectory.
    params.status = (constraints.check(params)) ? AimStatus.Possible : AimStatus.Impossible;
    return params;
  }
}

package frc.robot.aiming;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/**
 * One entry in the empirical calibration table used by {@link ToFAim}.
 *
 * <p>Calibration data is collected by placing the robot at known distances from the hub,
 * manually tuning the hood pitch and flywheel speed until the note enters consistently,
 * and recording those values here. {@link ToFAim} interpolates between entries at runtime
 * to find the best pitch and speed for any intermediate distance.
 *
 * <p>Two separate tables exist:
 * <ul>
 *   <li>{@link frc.robot.subsystems.shooter.ShooterConstants#scoringMeasurements} — distances and
 *       speeds calibrated for scoring into the hub.
 *   <li>{@link frc.robot.subsystems.shooter.ShooterConstants#feedingMeasurements} — calibrated for
 *       passing notes to a partner robot.
 * </ul>
 *
 * @param distance       Straight-line distance from the robot to the hub center (meters).
 * @param pitch          Hood angle at which this measurement was taken.
 * @param shooterControl Flywheel speed in rotations per second at which the shot was successful.
 *                       Stored as a raw {@code double} (RPS) rather than an {@code AngularVelocity}
 *                       for compatibility with the interpolation arithmetic in {@link ToFAim}.
 * @param time           Expected time-of-flight for a note shot at this distance (seconds).
 *                       Used by {@link frc.robot.aiming.LeadCompensator} to predict how far the
 *                       robot moves during flight.
 */
public record AimMeasurement(
      Distance distance,
      Rotation2d pitch,
      double shooterControl,
      Time time) {
}

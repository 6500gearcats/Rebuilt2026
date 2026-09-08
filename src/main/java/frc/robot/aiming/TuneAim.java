package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.aiming.AimParams.AimStatus;
import frc.robot.aiming.AimParams.SpeedControl;

/**
 * Manual aiming strategy that reads hood pitch and flywheel speed from SmartDashboard
 * sliders, bypassing the calibration table and physics solver.
 *
 * <p>Use this strategy during initial tuning sessions to find the correct pitch and speed
 * for a new shot distance without editing code:
 * <ol>
 *   <li>Switch to this strategy from {@link frc.robot.RobotContainer}.
 *   <li>Observe {@code "Distance"} on SmartDashboard to know the current shot range.
 *   <li>Adjust {@code "Aim Tuning/Pitch"} (degrees) and {@code "Aim Tuning/Output"} (RPS)
 *       until notes enter the hub consistently.
 *   <li>Record the values as a new {@link AimMeasurement} in
 *       {@link frc.robot.subsystems.shooter.ShooterConstants#scoringMeasurements}.
 * </ol>
 *
 * <p>Unlike {@link ToFAim} and {@link PhysicsAim}, this strategy always returns
 * {@link frc.robot.aiming.AimParams.AimStatus#Possible} — it never reports
 * {@code Impossible}, so the shooter will attempt a shot at whatever values are on
 * the dashboard, even if they are outside safe operating ranges.
 */
public class TuneAim implements AimStrategy {
  /**
   * Returns aim parameters driven by SmartDashboard sliders.
   *
   * @param aimTarget 3D pose of the scoring target (used only to compute yaw direction and
   *                  display distance — the pitch and speed come from SmartDashboard).
   * @param shooter   3D pose of the shooter exit point.
   * @param velocity  Ignored — lead compensation is not applied in manual tuning mode.
   */
  public AimParams update(Pose3d aimTarget, Pose3d shooter, Translation2d velocity) {
    Translation2d target = aimTarget.getTranslation().toTranslation2d();
    Translation2d start = shooter.getTranslation().toTranslation2d();

    Translation2d offset = target.minus(start);

    double distance = start.minus(target).getNorm();
    SmartDashboard.putNumber("Distance", distance);

    AimParams params = new AimParams(AimStatus.Possible);

    double pitch = SmartDashboard.getNumber("Aim Tuning/Pitch", 60.0);
    double output = SmartDashboard.getNumber("Aim Tuning/Output", 0);

    SmartDashboard.putNumber("Aim Tuning/Pitch", pitch);
    SmartDashboard.putNumber("Aim Tuning/Output", output);

    params.pitch = Rotation2d.fromDegrees(pitch);
    params.output = output;
    params.control = SpeedControl.MechanismControl;

    params.yaw = Rotation2d.fromRadians(Math.atan2(offset.getY(), offset.getX()));

    return params;
  }
}

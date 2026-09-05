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
 * An aim strategy that uses time-of-flight recursion to estimate the ideal shot parameters for the
 * robot. This is based on measurements experimentally obtained.
 */
public class ToFAim implements AimStrategy {
  static final double EPSILON = 1e-3;
  static final int ITERATIONS = 15;

  private final AimConstraints constraints;

  private final InterpolatingDoubleTreeMap timeMap;
  private final InterpolatingDoubleTreeMap speedMap;
  private final InterpolatingDoubleTreeMap pitchMap;

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

  public AimParams update(Pose3d aimTarget, Pose3d shooterPose, Translation2d shooterVelocity) {
    Translation2d target = aimTarget.getTranslation().toTranslation2d();
    Translation2d start = shooterPose.getTranslation().toTranslation2d();

    Translation2d afterShooting = start;

    AimStatus status = AimStatus.Impossible;

    double distance = start.minus(target).getNorm();
    double tof;

    for (int i = 0; i < ITERATIONS; i++) {
      distance = afterShooting.minus(target).getNorm();
      tof = timeMap.get(distance);
      Translation2d newAfterShooting = start.plus(shooterVelocity.times(tof));
      double error = newAfterShooting.minus(afterShooting).getNorm();
      if (error < EPSILON) {
        status = AimStatus.Possible;
        break;
      }

      afterShooting = newAfterShooting;
    }

    if (status == AimStatus.Impossible) {
      return AimParams.impossible();
    }

    double pitch = pitchMap.get(distance);
    double shooterControl = speedMap.get(distance);

    AimParams params = new AimParams();
    params.pitch = Rotation2d.fromDegrees(pitch);
    params.output = shooterControl;
    params.control = SpeedControl.MechanismControl;
    Translation2d finalOffset = target.minus(afterShooting);
    params.yaw = Rotation2d.fromRadians(Math.atan2(finalOffset.getY(), finalOffset.getX()));

    params.status = (constraints.check(params)) ? AimStatus.Possible : AimStatus.Impossible;
    return params;
  }
}

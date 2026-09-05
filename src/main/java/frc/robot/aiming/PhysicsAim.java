package frc.robot.aiming;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.aiming.AimParams.AimStatus;
import frc.robot.aiming.AimParams.SpeedControl;

public class PhysicsAim implements AimStrategy {
  private static final int ITERATIONS = 5;

  private final AimConstraints constraints;

  private final double minDescentVelocity;
  private final double maxDescentVelocity;

  public PhysicsAim(AimConstraints constraints, double minDescentVelocity,
      double maxDescentVelocity) {
    this.constraints = constraints;
    this.minDescentVelocity = minDescentVelocity;
    this.maxDescentVelocity = maxDescentVelocity;
  }

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

    Rotation2d towardsTarget = Rotation2d.fromRadians(Math.atan2(offset.getY(), offset.getX()));
    double diff = MathUtil.angleModulus(Math.abs(towardsTarget.minus(best.yaw).getRadians()));
    if (diff > 0.8 * Math.PI) {
      return AimParams.impossible();
    }

    best.control = SpeedControl.ProjectileVelocity;
    return best;
  }

  public static AimParams quicksolve(
      Translation3d offset,
      Translation2d robotVelocity,
      double finalDescentSpeed) {

    AimParams params = new AimParams();

    double dx = offset.getX();
    double dy = offset.getY();
    double dz = offset.getZ();

    double a = 0.5 * 9.81;
    double b = -finalDescentSpeed;
    double c = -dz;
    double discriminant = b * b - 4 * a * c;
    double t = (-b + Math.sqrt(discriminant)) / (2 * a);

    double vx = dx / t;
    double vy = dy / t;
    double vz = 9.81 * t - finalDescentSpeed;

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

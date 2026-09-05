package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.aiming.AimParams.AimStatus;
import frc.robot.aiming.AimParams.SpeedControl;

public class TuneAim implements AimStrategy {
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

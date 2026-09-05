package frc.robot.aiming;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * A class representing the physical constraints of the shooter.
 */
public record AimConstraints(
    Rotation2d minShooterAngle,
    Rotation2d maxShooterAngle,
    double maxOutput) {

  /** Returns whether the given aim parameters satisfy this constraint */
  public boolean check(AimParams params) {
    boolean outputOk = params.output <= maxOutput();
    boolean pitchOk = (params.pitch.getRadians() >= minShooterAngle.getRadians())
        && (params.pitch.getRadians() <= maxShooterAngle.getRadians());
    return outputOk && pitchOk;
  }
}

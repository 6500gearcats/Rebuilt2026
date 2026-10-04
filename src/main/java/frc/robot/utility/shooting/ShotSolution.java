package frc.robot.utility.shooting;

import edu.wpi.first.math.geometry.Pose2d;

/** Immutable snapshot shared by aiming, motor control, feed gating, and telemetry. */
public record ShotSolution(Pose2d targetPose, double distanceMeters, double effectiveDistanceMeters,
        double tofSeconds, double radialVelocityMetersPerSecond, double flywheelRps,
        double hoodRotations, double turretVelocityXMetersPerSecond,
        double turretVelocityYMetersPerSecond, boolean valid, String status) {

    /** Invalid solutions carry no motor setpoints and cannot authorize feeding. */
    public static ShotSolution invalid(Pose2d targetPose, double distanceMeters, String reason) {
        return new ShotSolution(targetPose, distanceMeters, distanceMeters, 0, 0, 0, 0, 0, 0, false, reason);
    }

    // Preserve readable accessor names used by existing aiming/shooting code.
    public Pose2d getTargetPose() { return targetPose; }
    public double getDistance() { return distanceMeters; }
    public double getEffectiveDistance() { return effectiveDistanceMeters; }
    public double getTimeOfFlight() { return tofSeconds; }
    public double getRadialVelocity() { return radialVelocityMetersPerSecond; }
    public double getFlywheelSpeed() { return flywheelRps; }
    public double getHoodRotations() { return hoodRotations; }
    public double getTurretVelocityX() { return turretVelocityXMetersPerSecond; }
    public double getTurretVelocityY() { return turretVelocityYMetersPerSecond; }
    public boolean isValid() { return valid; }
}

package frc.robot.utility.shooting;

/** One accepted stationary shot. TOF runs from ball release to hub entry. */
public record ShotSample(double distanceMeters, double flywheelRps,
        double hoodRotations, double tofSeconds) {
}

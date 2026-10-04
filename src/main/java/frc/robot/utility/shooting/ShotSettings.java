package frc.robot.utility.shooting;

/** The three coordinated settings interpolated from the same pair of samples. */
public record ShotSettings(double flywheelRps, double hoodRotations, double tofSeconds) {
}

package frc.robot.utility.shooting;

/**
 * Accepted hub shots, ordered by horizontal turret-base-to-hub-center distance.
 * Add only measured rows; keep trial IDs and results in docs/shooting-calibration.md.
 * The empty table intentionally disables automatic shooting during initial tuning.
 */
public final class ShotTable {
    private ShotTable() {}

    public static ShotSample[] samples() {
        return new ShotSample[] {
            // new ShotSample(distanceMeters, flywheelRps, hoodRotations, tofSeconds),
        };
    }
}

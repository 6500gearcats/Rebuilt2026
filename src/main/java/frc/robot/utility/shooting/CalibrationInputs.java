package frc.robot.utility.shooting;

import java.util.Locale;
import java.util.Optional;

/** Manual inputs; missing TOF is allowed for stationary tuning, never for an accepted row. */
public record CalibrationInputs(double distanceMeters, double flywheelRps,
        double hoodRotations, double tofSeconds) {
    public Optional<ShotSettings> manualSettings(double minHoodRotations, double maxHoodRotations) {
        if (!Double.isFinite(distanceMeters) || distanceMeters <= 0
                || !Double.isFinite(flywheelRps) || flywheelRps <= 0
                || !Double.isFinite(hoodRotations) || hoodRotations < minHoodRotations
                || hoodRotations > maxHoodRotations || !Double.isFinite(tofSeconds) || tofSeconds < 0) {
            return Optional.empty();
        }
        return Optional.of(new ShotSettings(flywheelRps, hoodRotations, tofSeconds));
    }

    /** Candidate only: the operator must still validate repeated batches and provenance. */
    public Optional<String> javaRow(double minHoodRotations, double maxHoodRotations) {
        if (manualSettings(minHoodRotations, maxHoodRotations).isEmpty() || tofSeconds <= 0) {
            return Optional.empty();
        }
        return Optional.of(String.format(Locale.ROOT, "new ShotSample(%.6f, %.6f, %.6f, %.6f),",
                distanceMeters, flywheelRps, hoodRotations, tofSeconds));
    }
}

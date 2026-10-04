package frc.robot.utility.shooting;

import java.util.Optional;

/** Linear interpolation of a single coordinated shot table, without hardware I/O. */
public final class ShotSolver {
    private final ShotSample[] samples;

    /**
     * Copies and validates the table. Zero rows is allowed for initial calibration;
     * one row cannot define a calibrated interval. Invalid data is rejected at startup.
     */
    public ShotSolver(ShotSample[] samples, double minHoodRotations, double maxHoodRotations) {
        if (samples == null || samples.length == 1
                || !Double.isFinite(minHoodRotations) || !Double.isFinite(maxHoodRotations)
                || minHoodRotations >= maxHoodRotations) {
            throw new IllegalArgumentException("Shot table needs zero or at least two rows and valid hood limits");
        }
        this.samples = samples.clone();
        double previousDistance = 0;
        for (ShotSample row : this.samples) {
            if (row == null || !Double.isFinite(row.distanceMeters())
                    || row.distanceMeters() <= previousDistance
                    || !Double.isFinite(row.flywheelRps()) || row.flywheelRps() <= 0
                    || !Double.isFinite(row.tofSeconds()) || row.tofSeconds() <= 0
                    || !Double.isFinite(row.hoodRotations())
                    || row.hoodRotations() < minHoodRotations || row.hoodRotations() > maxHoodRotations) {
                throw new IllegalArgumentException("Shot rows must be finite, increasing, positive, and within hood limits");
            }
            previousDistance = row.distanceMeters();
        }
    }

    /** Returns no settings for an empty table, invalid distance, or untested range. */
    public Optional<ShotSettings> solve(double distanceMeters) {
        if (!Double.isFinite(distanceMeters) || samples.length == 0
                || distanceMeters < samples[0].distanceMeters()
                || distanceMeters > samples[samples.length - 1].distanceMeters()) {
            return Optional.empty();
        }
        for (int i = 1; i < samples.length; i++) {
            ShotSample upper = samples[i];
            if (distanceMeters <= upper.distanceMeters()) {
                ShotSample lower = samples[i - 1];
                double fraction = (distanceMeters - lower.distanceMeters())
                        / (upper.distanceMeters() - lower.distanceMeters());
                return Optional.of(new ShotSettings(
                        interpolate(lower.flywheelRps(), upper.flywheelRps(), fraction),
                        interpolate(lower.hoodRotations(), upper.hoodRotations(), fraction),
                        interpolate(lower.tofSeconds(), upper.tofSeconds(), fraction)));
            }
        }
        return Optional.empty();
    }

    public boolean isCalibrated() { return samples.length >= 2; }

    private static double interpolate(double lower, double upper, double fraction) {
        return lower + fraction * (upper - lower);
    }
}

package frc.robot.utility.shooting;

/** Requires a healthy measurement within tolerance for a continuous dwell period. */
public final class SetpointReadiness {
    private final double tolerance;
    private final double dwellSeconds;
    private double sinceSeconds = Double.NaN;
    private double dwellTarget = Double.NaN;
    private boolean ready;

    public SetpointReadiness(double tolerance, double dwellSeconds) {
        this.tolerance = tolerance;
        this.dwellSeconds = dwellSeconds;
    }

    /**
     * Reset on a meaningful target change, including accumulated small changes.
     * Comparing with the dwell's starting target allows small moving-shot updates
     * without perpetually restarting the timer every 20 ms.
     */
    public boolean update(double target, double actual, boolean healthy, double nowSeconds) {
        if (!healthy || !Double.isFinite(target) || !Double.isFinite(actual)
                || !Double.isFinite(nowSeconds) || Math.abs(target - actual) > tolerance) {
            reset();
            return false;
        }
        if (Double.isNaN(sinceSeconds) || nowSeconds < sinceSeconds
                || Math.abs(target - dwellTarget) > tolerance) {
            sinceSeconds = nowSeconds;
            dwellTarget = target;
        }
        ready = nowSeconds - sinceSeconds >= dwellSeconds;
        return ready;
    }

    public boolean isReady() { return ready; }

    public void reset() {
        sinceSeconds = Double.NaN;
        dwellTarget = Double.NaN;
        ready = false;
    }
}

package frc.robot.utility.shooting;

/** Small, testable decisions used by both manual calibration and automatic shots. */
public final class ShotSafety {
    private ShotSafety() {}

    public static boolean canFeed(boolean enabled, boolean allowed, boolean valid,
            boolean aligned, boolean flywheelReady, boolean hoodReady) {
        return enabled && allowed && valid && aligned && flywheelReady && hoodReady;
    }

    /** At zero yaw, the shooter fires toward the robot's rear (+180 degrees). */
    public static double alignmentErrorDegrees(double targetBearingDegrees, double turretHeadingDegrees) {
        double errorRadians = Math.toRadians(targetBearingDegrees - turretHeadingDegrees - 180);
        return Math.toDegrees(Math.atan2(Math.sin(errorRadians), Math.cos(errorRadians)));
    }

    public static boolean isStationary(double vx, double vy, double omegaRadiansPerSecond) {
        return Double.isFinite(vx) && Double.isFinite(vy) && Double.isFinite(omegaRadiansPerSecond)
                && Math.hypot(vx, vy) <= 0.1 && Math.abs(omegaRadiansPerSecond) <= 0.1;
    }
}

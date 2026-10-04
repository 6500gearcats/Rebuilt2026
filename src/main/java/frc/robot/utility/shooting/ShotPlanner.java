package frc.robot.utility.shooting;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Adds the existing radial range correction and full velocity aim lead to a table. */
public final class ShotPlanner {
    private final ShotSolver solver;
    private ShotSolution solution = ShotSolution.invalid(new Pose2d(), 0, "Not updated");

    public ShotPlanner(ShotSolver solver) { this.solver = solver; }

    /** Getters never recompute: all consumers see the same cycle's snapshot. */
    public ShotSolution getSolution() { return solution; }

    public ShotSolution invalidate(Pose2d target, double distanceMeters, String reason) {
        solution = ShotSolution.invalid(target, distanceMeters, reason);
        return solution;
    }

    /** Manual tuning does not require a measured TOF or populated table. */
    public ShotSolution manual(Pose2d target, double distanceMeters, ShotSettings settings) {
        solution = new ShotSolution(target, distanceMeters, distanceMeters, settings.tofSeconds(), 0,
                settings.flywheelRps(), settings.hoodRotations(), 0, 0, true, "Manual calibration");
        return solution;
    }

    /**
     * Positive radial velocity means moving toward the target. The stationary-shot
     * equivalent range is distance - radialVelocity * TOF(range). Lateral velocity
     * remains in the aim lead, preserving the previous approximation.
     */
    public ShotSolution update(Pose2d turretPose, Pose2d target, Translation2d turretVelocity,
            boolean motionEnabled) {
        Translation2d vector = target.getTranslation().minus(turretPose.getTranslation());
        double distance = vector.getNorm();
        if (!Double.isFinite(distance) || distance < 1e-9
                || !Double.isFinite(turretVelocity.getX()) || !Double.isFinite(turretVelocity.getY())) {
            return invalidate(target, distance, "Invalid range or velocity");
        }
        if (!solver.isCalibrated()) {
            return invalidate(target, distance, "Shot table is empty");
        }
        double radialVelocity = motionEnabled
                ? turretVelocity.getX() * vector.getX() / distance
                        + turretVelocity.getY() * vector.getY() / distance
                : 0;
        double effectiveDistance = distance;
        boolean converged = !motionEnabled;
        if (motionEnabled) {
            for (int i = 0; i < 20; i++) {
                Optional<ShotSettings> settings = solver.solve(effectiveDistance);
                if (settings.isEmpty()) {
                    return invalidate(target, distance, "Effective distance outside calibrated range");
                }
                double nextDistance = distance - radialVelocity * settings.get().tofSeconds();
                converged = Math.abs(nextDistance - effectiveDistance) <= 0.001;
                effectiveDistance = nextDistance;
                if (converged) { break; }
            }
        }
        if (!converged) {
            return invalidate(target, distance, "Motion solution did not converge");
        }
        // Re-query once at the FINAL distance so speed, hood, and TOF agree.
        Optional<ShotSettings> result = solver.solve(effectiveDistance);
        if (result.isEmpty()) {
            return invalidate(target, distance, solver.isCalibrated()
                    ? "Distance outside calibrated range" : "Shot table is empty");
        }
        ShotSettings settings = result.get();
        Translation2d velocity = motionEnabled ? turretVelocity : new Translation2d();
        Pose2d aim = new Pose2d(target.getTranslation().minus(velocity.times(settings.tofSeconds())),
                new Rotation2d());
        solution = new ShotSolution(aim, distance, effectiveDistance, settings.tofSeconds(), radialVelocity,
                settings.flywheelRps(), settings.hoodRotations(), velocity.getX(), velocity.getY(), true, "Ready solution");
        return solution;
    }
}

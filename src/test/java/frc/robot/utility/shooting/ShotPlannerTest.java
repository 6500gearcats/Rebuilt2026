package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class ShotPlannerTest {
    private ShotPlanner planner() {
        return new ShotPlanner(new ShotSolver(new ShotSample[] {
            new ShotSample(1, 40, 0.1, 1), new ShotSample(5, 80, 0.5, 5)
        }, 0.012, 0.673));
    }

    @Test
    void sharedSnapshotUsesFinalDistanceForEverySetting() {
        var planner = planner();
        var shot = planner.update(new Pose2d(), new Pose2d(3, 0, new Rotation2d()),
                new Translation2d(0.25, 0), true);
        assertTrue(shot.isValid());
        // d = 3 - 0.25*d => d = 2.4. TOF equals distance in this synthetic table.
        assertEquals(2.4, shot.getEffectiveDistance(), 0.001);
        assertEquals(shot.getEffectiveDistance(), shot.getTimeOfFlight(), 1e-9);
        assertEquals(54, shot.getFlywheelSpeed(), 0.02);
        assertEquals(0.24, shot.getHoodRotations(), 0.0002);
        assertEquals(3 - 0.25 * shot.getTimeOfFlight(), shot.getTargetPose().getX(), 1e-9);
        assertSame(shot, planner.getSolution());
        assertSame(shot, planner.getSolution());
    }

    @Test
    void lateralVelocityLeadsAimWithoutChangingRadialDistance() {
        var shot = planner().update(new Pose2d(), new Pose2d(3, 0, new Rotation2d()),
                new Translation2d(0, 0.2), true);
        assertEquals(3, shot.getEffectiveDistance(), 1e-9);
        assertEquals(-0.6, shot.getTargetPose().getY(), 1e-9);
    }

    @Test
    void failedConvergenceAndOutOfRangeCannotReuseLastValidSolution() {
        var planner = new ShotPlanner(new ShotSolver(new ShotSample[] {
            new ShotSample(1, 40, 0.1, 0.1), new ShotSample(2, 50, 0.2, 0.2),
            new ShotSample(3, 60, 0.3, 2), new ShotSample(4, 70, 0.4, 2.1)
        }, 0.012, 0.673));
        assertTrue(planner.update(new Pose2d(), new Pose2d(3, 0, new Rotation2d()),
                new Translation2d(), false).isValid());
        var failed = planner.update(new Pose2d(), new Pose2d(4, 0, new Rotation2d()),
                new Translation2d(1, 0), true);
        assertFalse(failed.isValid());
        assertEquals("Motion solution did not converge", failed.status());
        assertEquals(0, failed.getFlywheelSpeed());
        assertSame(failed, planner.getSolution());
        assertFalse(planner.update(new Pose2d(), new Pose2d(8, 0, new Rotation2d()),
                new Translation2d(), false).isValid());
    }

    @Test
    void manualCalibrationWorksWithoutTableAndReturningToAutomaticDoesNotKeepManualSettings() {
        var planner = new ShotPlanner(new ShotSolver(new ShotSample[0], 0.012, 0.673));
        var inputs = new CalibrationInputs(3, 50, 0.2, 0);
        var manual = planner.manual(new Pose2d(3, 0, new Rotation2d()), 3,
                inputs.manualSettings(0.012, 0.673).orElseThrow());
        assertTrue(manual.isValid());
        assertEquals(0, manual.getTimeOfFlight());
        var automatic = planner.update(new Pose2d(), manual.getTargetPose(), new Translation2d(), false);
        assertFalse(automatic.isValid());
        assertEquals(0, automatic.getFlywheelSpeed());
        assertFalse(planner.invalidate(new Pose2d(), 0, "Missing target").isValid());
    }

    @Test
    void rejectsNonfiniteVelocityAndDegenerateRange() {
        assertFalse(planner().update(new Pose2d(), new Pose2d(3, 0, new Rotation2d()),
                new Translation2d(Double.NaN, 0), true).isValid());
        assertFalse(planner().update(new Pose2d(), new Pose2d(), new Translation2d(), true).isValid());
    }
}

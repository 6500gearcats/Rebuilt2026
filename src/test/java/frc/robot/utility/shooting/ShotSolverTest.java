package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ShotSolverTest {
    // Synthetic fixtures ONLY; these are not robot calibration values.
    private ShotSample[] rows() {
        return new ShotSample[] {new ShotSample(2, 40, 0.1, 0.5), new ShotSample(4, 60, 0.3, 1.0)};
    }

    @Test
    void interpolatesAllSettingsWithOneFractionAndKeepsEndpoints() {
        var solver = new ShotSolver(rows(), 0.012, 0.673);
        var quarter = solver.solve(2.5).orElseThrow();
        assertEquals(45, quarter.flywheelRps(), 1e-9);
        assertEquals(0.15, quarter.hoodRotations(), 1e-9);
        assertEquals(0.625, quarter.tofSeconds(), 1e-9);
        assertEquals(new ShotSettings(40, 0.1, 0.5), solver.solve(2).orElseThrow());
        assertEquals(new ShotSettings(60, 0.3, 1), solver.solve(4).orElseThrow());
    }

    @Test
    void neverExtrapolatesAndCopiesCallerArray() {
        var data = rows();
        var solver = new ShotSolver(data, 0.012, 0.673);
        data[0] = new ShotSample(2, 90, 0.6, 9);
        assertEquals(40, solver.solve(2).orElseThrow().flywheelRps());
        for (double distance : new double[] {0, 1.999, 4.001, Double.NaN, Double.POSITIVE_INFINITY}) {
            assertTrue(solver.solve(distance).isEmpty());
        }
        var empty = new ShotSolver(new ShotSample[0], 0.012, 0.673);
        assertFalse(empty.isCalibrated());
        assertTrue(empty.solve(3).isEmpty());
    }

    @Test
    void rejectsInvalidTablesInsteadOfSilentlySortingOrOverwriting() {
        ShotSample[][] invalid = {
            null, {rows()[0]}, {rows()[0], rows()[0]}, {rows()[1], rows()[0]},
            {null, rows()[1]},
            {new ShotSample(Double.NaN, 40, 0.1, 0.5), rows()[1]},
            {new ShotSample(-1, 40, 0.1, 0.5), rows()[1]},
            {new ShotSample(2, 0, 0.1, 0.5), rows()[1]},
            {new ShotSample(2, Double.POSITIVE_INFINITY, 0.1, 0.5), rows()[1]},
            {new ShotSample(2, 40, -0.1, 0.5), rows()[1]},
            {new ShotSample(2, 40, 0.7, 0.5), rows()[1]},
            {new ShotSample(2, 40, Double.NaN, 0.5), rows()[1]},
            {new ShotSample(2, 40, 0.1, 0), rows()[1]},
            {new ShotSample(2, 40, 0.1, Double.NaN), rows()[1]}
        };
        for (ShotSample[] data : invalid) {
            assertThrows(IllegalArgumentException.class, () -> new ShotSolver(data, 0.012, 0.673));
        }
    }
}

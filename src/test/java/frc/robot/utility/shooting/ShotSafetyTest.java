package frc.robot.utility.shooting;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class ShotSafetyTest {
    @Test
    void everyGateIsRequiredIncludingAtCloseRange() {
        assertTrue(ShotSafety.canFeed(true, true, true, true, true, true));
        for (int i = 0; i < 6; i++) {
            boolean[] gates = {true, true, true, true, true, true};
            gates[i] = false;
            assertFalse(ShotSafety.canFeed(gates[0], gates[1], gates[2], gates[3], gates[4], gates[5]));
        }
        assertEquals(0, ShotSafety.alignmentErrorDegrees(180, 0), 1e-9);
        assertEquals(0, ShotSafety.alignmentErrorDegrees(-170, 10), 1e-9);
        assertEquals(2, ShotSafety.alignmentErrorDegrees(-178, 0), 1e-9);
        assertTrue(ShotSafety.isStationary(0.01, 0.01, 0.01));
        assertFalse(ShotSafety.isStationary(0, 0, 0.2));
        assertFalse(ShotSafety.isStationary(Double.NaN, 0, 0));
    }

    @Test
    void readinessRequiresDwellAndDropsOnErrorFaultStopOrLargeTargetChange() {
        var ready = new SetpointReadiness(2, 0.08);
        assertFalse(ready.update(50, 49, true, 0));
        assertFalse(ready.update(50, 49, true, 0.079));
        assertTrue(ready.update(50, 49, true, 0.081));
        assertFalse(ready.update(50, 47, true, 0.09));
        assertFalse(ready.update(50, 50, true, 0.1));
        assertTrue(ready.update(50, 50, true, 0.181));
        assertFalse(ready.update(55, 55, true, 0.2));
        assertTrue(ready.update(55, 55, true, 0.281));
        assertFalse(ready.update(55, 55, false, 0.3));
        ready.reset();
        assertFalse(ready.isReady());
    }

    @Test
    void smallMovingUpdatesCanSettleButAccumulatedChangesRestartDwell() {
        var ready = new SetpointReadiness(2, 0.08);
        assertFalse(ready.update(50, 50, true, 0));
        assertFalse(ready.update(50.5, 50.5, true, 0.04));
        assertTrue(ready.update(51, 51, true, 0.09));
        assertFalse(ready.update(52.1, 52.1, true, 0.1));
    }

    @Test
    void candidateRowsRequireMeasuredFlightTime() {
        assertTrue(new CalibrationInputs(3, 50, 0.2, 0).manualSettings(0.012, 0.673).isPresent());
        assertTrue(new CalibrationInputs(3, 50, 0.2, 0).javaRow(0.012, 0.673).isEmpty());
        assertEquals("new ShotSample(3.000000, 50.000000, 0.200000, 0.750000),",
                new CalibrationInputs(3, 50, 0.2, 0.75).javaRow(0.012, 0.673).orElseThrow());
        assertTrue(new CalibrationInputs(3, 50, 0.7, 0.75).manualSettings(0.012, 0.673).isEmpty());
        assertTrue(new CalibrationInputs(0, 50, 0.2, 0.75).manualSettings(0.012, 0.673).isEmpty());
    }
}

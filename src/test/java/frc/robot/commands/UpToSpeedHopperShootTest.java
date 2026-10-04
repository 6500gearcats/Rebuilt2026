package frc.robot.commands;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class UpToSpeedHopperShootTest {
    @Test
    void stopsImmediatelyWhenGateDropsAndLogsOnlyFeedTransitions() {
        boolean[] ready = {false};
        boolean[] motorRunning = {false};
        int[] loggedStarts = {0};
        var command = new UpToSpeedHopperShoot(() -> ready[0], () -> motorRunning[0] = true,
                () -> motorRunning[0] = false, () -> loggedStarts[0]++);
        command.initialize();
        command.execute();
        assertFalse(motorRunning[0]);
        ready[0] = true;
        command.execute();
        command.execute();
        assertTrue(motorRunning[0]);
        assertEquals(1, loggedStarts[0]);
        ready[0] = false;
        command.execute();
        assertFalse(motorRunning[0]);
        ready[0] = true;
        command.execute();
        assertEquals(2, loggedStarts[0]);
        command.end(true);
        assertFalse(motorRunning[0]);
        command.initialize();
        command.execute();
        assertEquals(3, loggedStarts[0]);
    }
}

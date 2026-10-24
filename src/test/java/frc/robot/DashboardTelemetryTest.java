package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class DashboardTelemetryTest {
    private boolean enabled;
    private double seconds;

    private DashboardTelemetry.DebugPublisher publisher() {
        return new DashboardTelemetry.DebugPublisher(() -> enabled, () -> seconds);
    }

    @Test
    void disabledDebugNeverPublishes() {
        var publisher = publisher();
        assertFalse(publisher.shouldPublish());
        seconds = 100;
        assertFalse(publisher.shouldPublish());
    }

    @Test
    void rapidControlUpdatesPublishAtMostTenTimesPerSecond() {
        var publisher = publisher();
        enabled = true;
        assertTrue(publisher.shouldPublish());
        for (int millisecond = 1; millisecond < 100; millisecond++) {
            seconds = millisecond / 1000.0;
            assertFalse(publisher.shouldPublish());
        }
        seconds = 0.1;
        assertTrue(publisher.shouldPublish());
        assertFalse(publisher.shouldPublish());
    }

    @Test
    void disablingStopsUpdatesAndReenablingPublishesImmediately() {
        var publisher = publisher();
        enabled = true;
        assertTrue(publisher.shouldPublish());
        enabled = false;
        seconds = 0.02;
        assertFalse(publisher.shouldPublish());
        enabled = true;
        assertTrue(publisher.shouldPublish());
    }

    @Test
    void independentOwnersDoNotSuppressEachOther() {
        var aiming = publisher();
        var drive = publisher();
        enabled = true;
        assertTrue(aiming.shouldPublish());
        assertTrue(drive.shouldPublish());
        assertFalse(aiming.shouldPublish());
        assertFalse(drive.shouldPublish());
    }
}

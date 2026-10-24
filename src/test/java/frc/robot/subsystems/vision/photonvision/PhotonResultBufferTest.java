package frc.robot.subsystems.vision.photonvision;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

class PhotonResultBufferTest {
    @Test
    void gettersDoNotReadOrConsumeCameraFrames() {
        var reads = new AtomicInteger();
        var frame = new PhotonPipelineResult();
        var buffer = new PhotonResultBuffer(() -> {
            reads.incrementAndGet();
            return List.of(frame);
        });
        assertFalse(buffer.latest().hasTargets());
        assertTrue(buffer.takeUnread().isEmpty());
        assertEquals(0, reads.get());

        buffer.update();
        assertSame(frame, buffer.latest());
        assertSame(frame, buffer.latest());
        assertEquals(List.of(frame), buffer.takeUnread());
        assertEquals(1, reads.get());
    }

    @Test
    void everyFrameIsAvailableForFusionExactlyOnceInOrder() {
        var first = new PhotonPipelineResult();
        var second = new PhotonPipelineResult();
        var buffer = new PhotonResultBuffer(() -> List.of(first, second));
        buffer.update();
        assertSame(second, buffer.latest());
        assertEquals(List.of(first, second), buffer.takeUnread());
        assertTrue(buffer.takeUnread().isEmpty());
        assertSame(second, buffer.latest());
    }

    @Test
    void loopWithoutNewFramesKeepsSnapshotWithoutReplayingPoseData() {
        var frame = new PhotonPipelineResult();
        var reads = new AtomicInteger();
        var buffer = new PhotonResultBuffer(() ->
                reads.getAndIncrement() == 0 ? List.of(frame) : List.of());
        buffer.update();
        assertEquals(List.of(frame), buffer.takeUnread());
        buffer.update();
        assertTrue(buffer.takeUnread().isEmpty());
        assertSame(frame, buffer.latest());
        assertEquals(2, reads.get());
    }

    @Test
    void newFrameWithoutTargetsReplacesThePreviousSnapshot() {
        var first = new PhotonPipelineResult(1, 1000, 2000, 0, List.of(new PhotonTrackedTarget()));
        assertTrue(first.hasTargets());
        var empty = new PhotonPipelineResult();
        var reads = new AtomicInteger();
        var buffer = new PhotonResultBuffer(() ->
                List.of(reads.getAndIncrement() == 0 ? first : empty));
        buffer.update();
        buffer.takeUnread();
        buffer.update();
        assertSame(empty, buffer.latest());
        assertFalse(buffer.latest().hasTargets());
        assertEquals(List.of(empty), buffer.takeUnread());
    }
}

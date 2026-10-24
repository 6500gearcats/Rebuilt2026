package frc.robot.subsystems.vision.photonvision;

import java.util.List;
import java.util.function.Supplier;

import org.photonvision.targeting.PhotonPipelineResult;

/** One camera read per loop, with a shared snapshot for all target getters. */
final class PhotonResultBuffer {
    private final Supplier<List<PhotonPipelineResult>> reader;
    private PhotonPipelineResult latest = new PhotonPipelineResult();
    private List<PhotonPipelineResult> unread = List.of();

    PhotonResultBuffer(Supplier<List<PhotonPipelineResult>> reader) {
        this.reader = reader;
    }

    void update() {
        unread = List.copyOf(reader.get());
        if (!unread.isEmpty()) {
            latest = unread.get(unread.size() - 1);
        }
    }

    PhotonPipelineResult latest() {
        return latest;
    }

    /** Pose consumers may consume each frame only once, independently of target getters. */
    List<PhotonPipelineResult> takeUnread() {
        List<PhotonPipelineResult> results = unread;
        unread = List.of();
        return results;
    }
}

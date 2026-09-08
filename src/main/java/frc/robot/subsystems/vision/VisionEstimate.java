package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Thin wrapper around PhotonVision's {@link EstimatedRobotPose} that exposes only what
 * the {@link frc.robot.subsystems.vision.Vision} subsystem needs: a 2D field pose
 * and the image-capture timestamp.
 *
 * <p>Decouples the Vision subsystem from PhotonVision types, so the interface
 * (and tests) don't need to depend on the PhotonVision API.
 */
public class VisionEstimate {
    EstimatedRobotPose pose;

    /**
     * @param pose The raw PhotonVision estimate to wrap.
     */
    public VisionEstimate(EstimatedRobotPose pose) {
        this.pose = pose;
    }

    /**
     * Returns the estimated robot pose projected onto the 2D field plane.
     *
     * @return the estimated {@link Pose2d}, or {@code null} if no estimate is available.
     */
    public Pose2d getPose() {
        if (pose != null) {
            return pose.estimatedPose.toPose2d();
        }
        return null;
    }

    /**
     * Returns the image-capture timestamp for this estimate.
     * Used by {@code SwerveDrivePoseEstimator} to apply the estimate at the correct
     * point in time rather than at the current loop cycle.
     *
     * @return timestamp in seconds (FPGA time), or {@code 0} if unavailable.
     */
    public double getTimestamp() {
        if (pose != null) {
            return pose.timestampSeconds;
        }
        return 0;
    }
}

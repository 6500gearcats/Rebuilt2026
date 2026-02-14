package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;
import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Wrapper that normalizes pose estimates from PhotonVision and Limelight.
 */
public class VisionEstimate {
    PoseEstimate poseEstimate;
    EstimatedRobotPose pose;

    public VisionEstimate(EstimatedRobotPose pose) {
        this.pose = pose;
    }

    public VisionEstimate(PoseEstimate poseEstimate) {
        this.poseEstimate = poseEstimate;
    }

    public Pose2d getPose() {
        if (pose != null) {
            return pose.estimatedPose.toPose2d();
        } else if (poseEstimate != null) {
            return poseEstimate.pose;
        }
        return null;
    }

    public double getTimestamp() {
        if (pose != null) {
            return pose.timestampSeconds;
        } else if (poseEstimate != null) {
            return poseEstimate.timestampSeconds;
        }
        return 0;
    }
}
package frc.robot.subsystems.vision;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;

/** Wrapper for PhotonVision pose estimates passed into the Vision subsystem. */
public class VisionEstimate {
    EstimatedRobotPose pose;

    public VisionEstimate(EstimatedRobotPose pose) {
        this.pose = pose;
    }

    public Pose2d getPose() {
        if (pose != null) {
            return pose.estimatedPose.toPose2d();
        }
        return null;
    }

    public double getTimestamp() {
        if (pose != null) {
            return pose.timestampSeconds;
        }
        return 0;
    }
}

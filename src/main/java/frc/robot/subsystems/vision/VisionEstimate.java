package frc.robot.subsystems.vision;

import frc.robot.subsystems.vision.limelight.LimelightHelpers.PoseEstimate;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.math.Matrix;

/**
 * Wrapper that normalizes pose estimates from PhotonVision and Limelight.
 */
public class VisionEstimate {
    PoseEstimate poseEstimate;
    EstimatedRobotPose pose;
    private final Matrix<N3, N1> stdDevs;

    public VisionEstimate(
            EstimatedRobotPose pose,
            Matrix<N3, N1> stdDevs) {
        this.pose = pose;
        this.stdDevs = stdDevs;
    }

    public VisionEstimate(PoseEstimate poseEstimate) {
        this.poseEstimate = poseEstimate;
        this.stdDevs = null;
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
    public Optional<Matrix<N3, N1>> getStdDevs() {
        return Optional.ofNullable(stdDevs);
    }
}
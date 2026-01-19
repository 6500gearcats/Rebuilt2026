package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.vision.Vision;

public final class RobotStateMachine {
    private RobotState state = RobotState.IDLE;

    // Latest known robot pose. This is always kept in sync with Vision when
    // available.
    private Pose2d pose = new Pose2d();
    private Supplier<Pose2d> visionPoseSupplier;

    /**
     * Bind the vision subsystem so this state machine can always fetch the latest
     * pose.
     */
    public void bindVision(Vision vision) {
        if (vision != null) {
            this.visionPoseSupplier = vision::getEstimatedPose;
        }

    }

    /**
     * Bind a pose supplier (for tests or alternate vision providers).
     */
    public void bindVisionPoseSupplier(Supplier<Pose2d> poseSupplier) {
        this.visionPoseSupplier = poseSupplier;
    }

    /**
     * Get the latest robot pose, refreshing from the vision estimator when present.
     */
    public Pose2d getPose() {
        refreshPoseFromVision();
        return pose;
    }

    /**
     * Manually set the cached robot pose (useful for initializing or tests).
     */
    public void setPose(Pose2d newPose) {
        if (newPose != null) {
            this.pose = newPose;
        }
    }

    /**
     * Update state and refresh pose from vision.
     */
    public void setState(RobotState next) {
        if (next == state)
            return;
        refreshPoseFromVision();
        update(next);
    }

    public void update(RobotState s) {
        switch (s) {
            case INTAKING:
                state = RobotState.INTAKING;
                break;
            case FEEDING:
                state = RobotState.FEEDING;
                break;
            case SHOOTING:
                state = RobotState.SHOOTING;
                break;
            case AUTO:
                state = RobotState.AUTO;
                break;
            case DISABLED:
                state = RobotState.DISABLED;
                break;
            default:
                break;
        }
    }

    /**
     * Pull the latest pose from the bound vision supplier and cache it locally.
     */
    private void refreshPoseFromVision() {
        if (visionPoseSupplier != null) {
            Pose2d latest = visionPoseSupplier.get();
            if (latest != null) {
                pose = latest;
            }
        }
    }

    enum RobotState {
        IDLE, INTAKING, FEEDING, SHOOTING, AUTO, DISABLED
    }
}

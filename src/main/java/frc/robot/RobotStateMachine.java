package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.Vision;

public final class RobotStateMachine {
    private static RobotStateMachine instance;

    private RobotState state = RobotState.INACTIVE;

    private Pose2d pose = new Pose2d();
    private Supplier<Pose2d> visionPoseSupplier;
    private FieldZone currentZone = FieldZone.ALLIANCE;

    private RobotStateMachine() {
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
    }

    public static RobotStateMachine getInstance() {
        if (instance == null) {
            instance = new RobotStateMachine();
        }
        return instance;
    }

    public void periodic() {
        refreshPoseFromVision();
        currentZone = checkZone();
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
    }

    public FieldZone getCurrentZone() {
        return currentZone;
    }

    public void setCurrentZone(FieldZone currentZone) {
        this.currentZone = currentZone;
    }

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

    public RobotState getState() {
        return state;
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
            case ACTIVE:
                state = RobotState.ACTIVE;
                break;
            case INACTIVE:
                state = RobotState.INACTIVE;
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

    public FieldZone checkZone() {
        // < 4.52 m is the blue alliance's trench, > 11.63 m is the red alliance's
        // trench, and in between is the neutral zone
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
        if (pose.getX() < 4.52) {
            return alliance.equals(Alliance.Blue) ? FieldZone.ALLIANCE : FieldZone.OPPONENT;
        } else if (pose.getX() > 11.63) {
            return alliance.equals(Alliance.Red) ? FieldZone.ALLIANCE : FieldZone.OPPONENT;
        } else {
            return FieldZone.NEUTRAL;
        }
    }

    enum RobotState {
        ACTIVE, INACTIVE
    }

    enum FieldZone {
        ALLIANCE, NEUTRAL, OPPONENT
    }
}

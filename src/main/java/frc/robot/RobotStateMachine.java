package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.vision.Vision;

/**
 * Singleton state machine that tracks robot state, pose, and field zone.
 */
public final class RobotStateMachine {
    private static RobotStateMachine instance;

    private RobotState state = RobotState.INACTIVE;

    private Pose2d turretPose = new Pose2d();

    private Pose2d pose = new Pose2d();
    private Supplier<Pose2d> visionPoseSupplier;
    private FieldZone currentZone = FieldZone.ALLIANCE;
    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getTable("StateMachinePose")
            .getStructTopic("stateMachinePose", Pose2d.struct)
            .publish();

    private final StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault()
    .getTable("StateMachinePose")
    .getStructTopic("TuuretPose", Pose2d.struct)
    .publish();

    private RobotStateMachine() {
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
    }

    /**
     * Returns the shared state machine instance.
     *
     * @return singleton instance
     */
    public static RobotStateMachine getInstance() {
        if (instance == null) {
            instance = new RobotStateMachine();
        }
        return instance;
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    /**
     * Updates pose, field zone, and publishes telemetry.
     */
    public void periodic() {
        refreshPoseFromVision();
        currentZone = checkZone();
        posePublisher.set(pose);
        turretPose = new Pose2d(pose.getX() - 0.1524, pose.getY() + 0.0635, new Rotation2d(0)).rotateAround(pose.getTranslation(), pose.getRotation());
        turretPosePublisher.set(turretPose);
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
    }

    /**
     * Returns the currently computed field zone.
     *
     * @return field zone classification
     */
    public FieldZone getCurrentZone() {
        return currentZone;
    }

    /**
     * Sets the current field zone override.
     *
     * @param currentZone new field zone
     */
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

    /**
     * Returns the current robot state.
     *
     * @return current state enum
     */
    public RobotState getState() {
        return state;
    }

    /**
     * Update state and refresh pose from vision.
     */
    /**
     * Requests a transition to the specified state.
     *
     * @param next next state to apply
     */
    public void setState(RobotState next) {
        if (next == state)
            return;
        refreshPoseFromVision();
        update(next);
    }

    /**
     * Applies the requested state transition.
     *
     * @param s state to apply
     */
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

    /**
     * Determines the field zone based on the current pose and alliance.
     *
     * @return field zone classification
     */
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

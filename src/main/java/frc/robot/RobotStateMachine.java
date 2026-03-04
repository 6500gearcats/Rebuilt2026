package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.Vision;
import frc.robot.utility.RangeFinder;

/**
 * Singleton state machine that tracks robot state, pose, and field zone.
 */
public final class RobotStateMachine {
    private static RobotStateMachine instance;

    private RobotState state = RobotState.INACTIVE;
    private String gameData = "";
    private boolean gotData = false;

    private Pose2d turretPose = new Pose2d();

    private Vision m_vision;

    public static Pose3d Tag_POSE2D;

    public static Pose2d HubPose;

    private Pose2d targetPose = new Pose2d();

    private Pose2d pose = new Pose2d();
    private FieldZone currentZone = FieldZone.ALLIANCE;

    private CommandSwerveDrivetrain drivetrain;

    private Alliance alliance = Alliance.Blue; // Default

    private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getTable("StateMachine")
            .getStructTopic("RobotPose", Pose2d.struct)
            .publish();

    private final StructPublisher<Pose2d> turretPosePublisher = NetworkTableInstance.getDefault()
            .getTable("StateMachine")
            .getStructTopic("TurretPose", Pose2d.struct)
            .publish();

    private final StructPublisher<Pose2d> targetPosePublisher = NetworkTableInstance.getDefault()
            .getTable("StateMachine")
            .getStructTopic("TargetPose", Pose2d.struct)
            .publish();

    private RobotStateMachine() {
        checkAlliance();
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
    }

    // 1.926m, Y: 1.524m Blue Allience Target Right
    // 14.7 m , 2.29 m Red alliance right

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

    /**
     * Updates pose, field zone, and publishes telemetry.
     */
    public void periodic() {
        checkAlliance();
        refreshPoseFromVision();
        currentZone = checkZone();
        posePublisher.set(pose);
        turretPose = new Pose2d(pose.getX() - 0.1524, pose.getY() + 0.0635, new Rotation2d(0))
                .rotateAround(pose.getTranslation(), pose.getRotation());
        turretPosePublisher.set(turretPose);
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
        SmartDashboard.putBoolean("IsActive", isActive());
        updateTargetPose();
    }

    private void checkAlliance() {
        if (getAlliance() == Alliance.Red) {
            Tag_POSE2D = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(10).get();
        } else {
            Tag_POSE2D = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get();
        }
        HubPose = Tag_POSE2D.toPose2d().transformBy(
                new Transform2d(Distance.ofRelativeUnits(-0.5842, Meter), Distance.ofBaseUnits(0, Meter),
                        new Rotation2d()));
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public Pose2d getTargetPose() {
        updateTargetPose();
        return targetPose;
    }

    public void updateTargetPose() {
        ChassisSpeeds speeds = getFieldSpeeds();
        ChassisSpeeds robotRelSpeeds = getChassisSpeeds();
        if (speeds == null && robotRelSpeeds == null) {
            return;
        }

        SmartDashboard.putNumber("VelX", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("VelY", speeds.vyMetersPerSecond);

        if (robotRelSpeeds.vxMetersPerSecond < 0 && robotRelSpeeds.vyMetersPerSecond < 0) {
            speeds = new ChassisSpeeds(speeds.vxMetersPerSecond * 1.3, speeds.omegaRadiansPerSecond * 1.3,
                    speeds.omegaRadiansPerSecond);
        }

        Pose2d nextPose = pose.plus(
                new Transform2d(speeds.vxMetersPerSecond * 2, speeds.vyMetersPerSecond * 2
                , new Rotation2d()));

        double distance = nextPose.getTranslation().getDistance(HubPose.getTranslation());
        double shotVelocity = RangeFinder.getShotVelocity(distance);

        // Apx launch angle is 65 deg
        double shootAng = Units.degreesToRadians(65);
        double dh = Units.inchesToMeters(56.375 - 19); // Delta height in inches
        double timeOfFlight = ((shotVelocity * Math.sin(shootAng))
                + Math.sqrt(Math.pow(shotVelocity, 2) * Math.pow(Math.sin(shootAng), 2)) - 2 * 9.8 * dh) / 9.8;

        Optional<Pose2d> bestPose = getBestPoseTarget();
        if (bestPose.isEmpty()) {
            return;
        }

        targetPose = bestPose.get().transformBy(new Transform2d(
                new Translation2d(speeds.vxMetersPerSecond * timeOfFlight,
                        speeds.vyMetersPerSecond * timeOfFlight),
                new Rotation2d()));

        targetPosePublisher.set(targetPose);
    }

    public ChassisSpeeds getFieldSpeeds() {
        if (drivetrain == null) {
            return null;
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), pose.getRotation());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drivetrain.getKinematics().toChassisSpeeds(
                drivetrain.getModule(0).getCurrentState(), drivetrain.getModule(1).getCurrentState(),
                drivetrain.getModule(2).getCurrentState(),
                drivetrain.getModule(3).getCurrentState());
    }

    public void resetVisionPose(Pose2d pose) {
        m_vision.resetVisionPose(pose);
    }

    public void bindDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
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
            m_vision = vision;
        }
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
        if (gameData.contains("R")) {
            if (alliance.equals(DriverStation.Alliance.Red)) {
                if (DriverStation.getMatchTime() > 130) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 105) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 80) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 55) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 30) {
                    setState(RobotState.INACTIVE);
                }
            } else {
                if (DriverStation.getMatchTime() > 130) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 105) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 80) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 55) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 30) {
                    setState(RobotState.ACTIVE);
                }
            }
        } else if (gameData.contains("B")) {
            if (alliance.equals(DriverStation.Alliance.Blue)) {
                if (DriverStation.getMatchTime() > 130) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 105) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 80) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 55) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 30) {
                    setState(RobotState.INACTIVE);
                }
            } else {
                if (DriverStation.getMatchTime() > 130) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 105) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 80) {
                    setState(RobotState.ACTIVE);
                } else if (DriverStation.getMatchTime() > 55) {
                    setState(RobotState.INACTIVE);
                } else if (DriverStation.getMatchTime() > 30) {
                    setState(RobotState.ACTIVE);
                }
            }
        }

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

    public void switchState() {
        if (getState() == RobotState.ACTIVE) {
            setState(RobotState.INACTIVE);
        } else {
            setState(RobotState.ACTIVE);
        }
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
        if (m_vision != null) {
            Pose2d latest = m_vision.getEstimatedPose();
            if (latest != null) {
                pose = latest;
            }
        }
    }

    /**
     * Determines the field zone based on the current pose and alliance.
     *
     * @return The {@link FieldZone} you are in, e.g, Allience or neutral
     */
    public FieldZone checkZone() {
        // < 4.52 m is the blue alliance's trench, > 11.63 m is the red alliance's
        // trench, and in between is the neutral zone
        Alliance alliance = getAlliance();
        if (pose.getX() < 4.52) {
            currentZone = alliance.equals(Alliance.Blue) ? FieldZone.ALLIANCE : FieldZone.OPPONENT;
            return currentZone;
        } else if (pose.getX() > 11.63) {
            currentZone = alliance.equals(Alliance.Red) ? FieldZone.ALLIANCE : FieldZone.OPPONENT;
            return currentZone;
        } else {
            if (pose.getY() > 4.2) {
                return FieldZone.NEUTRAL_BOTTOM;
            } else if (pose.getY() < 3.8) {
                return FieldZone.NEUTRAL_TOP;
            } else {
                return FieldZone.NEUTRAL_CENTER;
            }
        }
    }

    public String getGameData() {
        return gameData;
    }

    public void setGameData(String data) {
        gameData = data;
        gotData = true;
    }

    public boolean hasData() {
        return gotData;
    }

    public Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }

    public enum RobotState {
        ACTIVE, INACTIVE
    }

    public enum FieldZone {
        ALLIANCE, NEUTRAL_TOP, NEUTRAL_CENTER, NEUTRAL_BOTTOM, OPPONENT
    }

    public boolean isActive() {
        return getState() == RobotState.ACTIVE;
    }

    private Optional<Pose2d> getBestPoseTarget() {
        if (checkZone() == FieldZone.ALLIANCE) {
            return Optional.of(HubPose);
        } else {
            // return feed position
            if (getAlliance() == Alliance.Blue) {
                if (checkZone() == FieldZone.NEUTRAL_TOP) {
                    // top blue pose
                    return Optional.of(new Pose2d(2.26, 1.681, new Rotation2d()));
                } else if (checkZone() == FieldZone.NEUTRAL_BOTTOM) {
                    // bottom blue pose
                    return Optional.of(new Pose2d(2.26, 5.835, new Rotation2d()));
                }
            } else {
                if (checkZone() == FieldZone.NEUTRAL_BOTTOM) {
                    // bottom red pose
                    return Optional.of(new Pose2d(14.71, 5.835, new Rotation2d()));
                } else if (checkZone() == FieldZone.NEUTRAL_TOP) {
                    // top red pose
                    return Optional.of(new Pose2d(14.71, 1.681, new Rotation2d()));
                }
            }
        }
        return Optional.empty();
    }
}

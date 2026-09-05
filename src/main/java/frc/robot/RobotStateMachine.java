package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.aiming.AimConstraints;
import frc.robot.aiming.AimParams;
import frc.robot.aiming.LeadCompensator;
import frc.robot.aiming.ToFAim;
import frc.robot.util.OnboardLogger;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.Vision;

/**
 * Singleton that owns the robot's state, pose, aiming pipeline, and field-zone tracking.
 *
 * <h2>Why a Singleton?</h2>
 * Many subsystems and commands need access to the same robot pose and aim state (drive, vision,
 * turret, shooter, auto). A singleton eliminates the need to pass references through every
 * constructor and keeps the shared state in one place. The instance is created eagerly at class
 * load time (see {@link #instance}) so there is no thread-safety risk from lazy initialization.
 *
 * <h2>Aiming Pipeline</h2>
 * The full shot-computation chain runs on-demand via {@link #getAimParams()}:
 * <pre>
 *   getAimParams()
 *     → LeadCompensator.computeLeadTarget(hub, turretPose, fieldVelocity, m_tofAim)
 *         → m_tofAim.update(virtualTarget, turretPose, kZero)   [15-iter convergence]
 *         → AimParams { yaw, pitch, output, tof }
 *     → m_tofAim.update(virtualTarget, turretPose, kZero)       [final params]
 *     → AimParams (used by Turret.track and Shooter.shoot)
 * </pre>
 *
 * <h2>Periodic Structure</h2>
 * {@link #periodic()} has two tiers:
 * <ul>
 *   <li><b>Every loop (50 Hz):</b> pose update, field-zone check, turret pose calculation.
 *       These feed directly into the 20 ms control loop and must run each cycle.
 *   <li><b>10 Hz (rate-limited by {@code m_telemetryTimer}):</b> SmartDashboard writes, NT4
 *       struct publishes, LED color updates. Slower SmartDashboard writes reduce NT4 bandwidth
 *       and prevent loop overruns (see Stage 0 for the overrun history).
 * </ul>
 *
 * <h2>LED / Color State Machine</h2>
 * The {@link #getState()} method implements a schedule-driven LED pattern based on the match
 * timer and game-specific data from the Driver Station. Red flashes, green flashes, and solid
 * colors signal alliance-specific scoring windows to the drive team.
 */
public final class RobotStateMachine {
    private static final RobotStateMachine instance = new RobotStateMachine();

    private RobotState state = RobotState.ACTIVE;
    private String gameData = "";
    private boolean gotData = false;

    private static final AimConstraints kScoringConstraints = new AimConstraints(
        Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(80), 100.0);

    private boolean switching = false;
    private boolean switchingRed = false;
    private boolean switchingGreen = false;
    private boolean postedValue = false;
    private Color exampleColor;
    private Color whiteColor = new Color(237, 237, 237);
    private Color blackColor = new Color(49, 49, 49);
    private Color redColor = new Color(191, 0, 0);
    private Color greenColor = new Color(0, 191, 0);

    private Pose2d turretPose = new Pose2d();
    private Pose3d m_lastLeadTarget = new Pose3d();

    private Vision m_vision;
    private final Shooter m_Shooter = new Shooter(
        Constants.RobotConstants.currentMode == Constants.Mode.REAL
            ? new ShooterIOHardware()
            : new ShooterIOSim());
    private final ToFAim m_tofAim = new ToFAim(ShooterConstants.scoringMeasurements, kScoringConstraints);

    public static Pose3d Tag_POSE2D;

    public static Pose2d HubPose;

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

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController m_gunner = new XboxController(1);
    private final Timer m_telemetryTimer = new Timer();

    private RobotStateMachine() {
        checkAlliance();
        exampleColor = whiteColor;
        m_telemetryTimer.start();
        SmartDashboard.putString("Robot/State", state.toString());
        SmartDashboard.putString("Robot/FieldZone", currentZone.toString());
        AimParams.setupLogging(new OnboardLogger("Aiming"), this::getAimParams);
    }

    public Shooter getShooter() {
        return m_Shooter;
    }

    /**
     * Runs the full aiming pipeline and returns the current lead-compensated shot parameters.
     *
     * <p>Called every loop by:
     * <ul>
     *   <li>{@code Shooter.tracked()} (polled by the WPILib scheduler via the Trigger)
     *   <li>{@code Turret.track()} (inside the tracking run() command)
     *   <li>{@code periodic()} at 10 Hz for SmartDashboard telemetry
     * </ul>
     *
     * <h2>Pipeline</h2>
     * <ol>
     *   <li>Guard: if no AprilTag pose is available yet, return {@link AimParams#impossible()}.
     *   <li>Get field-relative velocity from drivetrain. Use zero if drivetrain is not yet bound
     *       (prevents NPE during simulation startup before drivetrain is wired).
     *   <li>{@link LeadCompensator#computeLeadTarget} iterates 5 times to find the virtual target
     *       pose that compensates for robot motion during ball flight. Caches result in
     *       {@code m_lastLeadTarget} for telemetry ({@code Aiming/LeadOffsetXM/YM}).
     *   <li>{@link ToFAim#update} computes final pitch, speed, yaw, and TOF from the virtual
     *       target distance. Called with {@link Translation2d#kZero} velocity because lead
     *       compensation was already handled by {@link LeadCompensator}.
     * </ol>
     *
     * @return shot parameters; status is {@link frc.robot.aiming.AimParams.AimStatus#Impossible}
     *         when no target is available, the distance is out of range, or the required angle
     *         violates {@code kScoringConstraints}
     */
    public AimParams getAimParams() {
        if (Tag_POSE2D == null) return AimParams.impossible();
        ChassisSpeeds fs = getFieldSpeeds();
        Translation2d velocity = (fs != null)
            ? new Translation2d(fs.vxMetersPerSecond, fs.vyMetersPerSecond)
            : Translation2d.kZero;
        Pose3d shooterPose = new Pose3d(turretPose);
        // Outer lead-compensation loop — shifts the hub by -velocity*tof to get a virtual target.
        // Passes kZero to ToFAim so lead is not double-counted inside ToFAim's own loop.
        m_lastLeadTarget = LeadCompensator.computeLeadTarget(Tag_POSE2D, shooterPose, velocity, m_tofAim);
        return m_tofAim.update(m_lastLeadTarget, shooterPose, Translation2d.kZero);
    }

    public boolean isShootReady() {
        return m_Shooter.tracked(() -> getAimParams()).getAsBoolean();
    }

    public CommandXboxController getDriver() {
        return joystick;
    }

    public XboxController getGunner() {
        return m_gunner;
    }

    // 1.926m, Y: 1.524m Blue Allience Target Right
    // 14.7 m , 2.29 m Red alliance right

    /**
     * Returns the shared state machine instance.
     *
     * @return singleton instance
     */
    public static RobotStateMachine getInstance() {
        return instance;
    }

    /**
     * Called by the WPILib framework every 20 ms (50 Hz). Updates all control-critical state
     * and publishes telemetry at a reduced rate to avoid loop overruns.
     *
     * <h2>Tier 1 — Every Loop (control-critical)</h2>
     * <ul>
     *   <li>Game-specific data: used by the LED state machine to know which scoring windows
     *       are active for this alliance.
     *   <li>Pose refresh: fuses the latest vision estimate into the odometry pose. Runs every
     *       loop so the aiming pipeline always has the freshest position.
     *   <li>Field zone: determines ALLIANCE / NEUTRAL / OPPONENT for game-state LED feedback.
     *   <li>Turret pose: recomputes the turret's field position from the current robot pose and
     *       a fixed offset. Used as the shooter origin in the aiming pipeline.
     * </ul>
     *
     * <h2>Tier 2 — 10 Hz (display only)</h2>
     * SmartDashboard and NT4 struct publishes do not affect control; they only provide
     * AdvantageScope / Shuffleboard visibility. Rate-limiting to 10 Hz reduces the NT4 update
     * flood that caused loop overruns before Stage 0.
     */
    public void periodic() {
        // --- Tier 1: control-critical, every loop ---
        gameData = DriverStation.getGameSpecificMessage();
        alliance = getAlliance();
        checkAlliance();
        refreshPoseFromVision();
        currentZone = checkZone();
        // Turret is mounted 0.1524 m behind and 0.0635 m to the left of robot center (field frame),
        // then rotated with the robot. This gives the actual field position of the launch point.
        turretPose = new Pose2d(pose.getX() - 0.1524, pose.getY() + 0.0635, new Rotation2d(0))
                .rotateAround(pose.getTranslation(), pose.getRotation());

        // --- Tier 2: display only, 10 Hz ---
        if (m_telemetryTimer.advanceIfElapsed(0.1)) {
            posePublisher.set(pose);
            turretPosePublisher.set(turretPose);
            newPostedValue();
            SmartDashboard.putBoolean("Robot/DriverConnected", joystick.isConnected());
            SmartDashboard.putBoolean("Robot/GunnerConnected", m_gunner.isConnected());
            SmartDashboard.putString("Robot/LEDColor", exampleColor.toHexString());
            SmartDashboard.putString("Robot/State", state.toString());
            SmartDashboard.putString("Robot/FieldZone", currentZone.toString());
            SmartDashboard.putBoolean("Robot/IsActive", isActive());
            SmartDashboard.putNumber("Robot/MatchTimeSec", DriverStation.getMatchTime());
            SmartDashboard.putNumber("Robot/DistToHubM", distToTag());
            SmartDashboard.putBoolean("Robot/IsFacingHub", isFacingHub());
            SmartDashboard.putBoolean("Robot/IsShootReady", isShootReady());
            if (Tag_POSE2D != null) {
                SmartDashboard.putNumber("Aiming/LeadOffsetXM", m_lastLeadTarget.getX() - Tag_POSE2D.getX());
                SmartDashboard.putNumber("Aiming/LeadOffsetYM", m_lastLeadTarget.getY() - Tag_POSE2D.getY());
            }
            ChassisSpeeds fieldSpeeds = getFieldSpeeds();
            if (fieldSpeeds != null) {
                SmartDashboard.putNumber("Robot/VelXMps", fieldSpeeds.vxMetersPerSecond);
                SmartDashboard.putNumber("Robot/VelYMps", fieldSpeeds.vyMetersPerSecond);
            }
        }
    }

    private void newPostedValue() {
        if (switchingRed) {
            if (exampleColor.equals(whiteColor) || exampleColor.equals(blackColor) || exampleColor.equals(greenColor)) {
                exampleColor = redColor;
            } else if (exampleColor.equals(redColor)) {
                exampleColor = blackColor;
            }
        } else if (switchingGreen) {
            if (exampleColor.equals(whiteColor) || exampleColor.equals(blackColor) || exampleColor.equals(redColor)) {
                exampleColor = greenColor;
            } else if (exampleColor.equals(greenColor)) {
                exampleColor = blackColor;
            }
        } else if (switching) {
            if (exampleColor.equals(whiteColor)) {
                exampleColor = blackColor;
            } else if (exampleColor.equals(blackColor)) {
                exampleColor = whiteColor;
            }
        }
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

    public Pose2d getHubPose() {
        return HubPose;
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public ChassisSpeeds getFieldSpeeds() {
        if (drivetrain == null) {
            return null;
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), pose.getRotation());
    }

    public ChassisSpeeds getChassisSpeeds() {
        if (drivetrain == null) return new ChassisSpeeds();
        return drivetrain.getState().Speeds;
    }

    public void resetVisionPose(Pose2d pose) {
        m_vision.resetVisionPose(pose);
    }

    // public void getVisionEst(String name) {
    // m_vision.getEstPoses(name);
    // }

    public void bindDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public double distToTag() {
        return pose.getTranslation().getDistance(HubPose.getTranslation());
    }

    public BooleanSupplier isFarEnough() {
        return () -> distToTag() > 4.2;
    }

    public boolean isFacingHub() {
        double dx = HubPose.getX() - pose.getX();
        double dy = HubPose.getY() - pose.getY();
        double targetAngle = Math.atan2(dy, dx);
        double delta = targetAngle - (pose.getRotation().getRadians() - Math.PI);
        delta = Math.atan2(Math.sin(delta), Math.cos(delta));
        double tolerance = Math.toRadians(20);

        return Math.abs(delta) < tolerance;
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
        double matchTime = DriverStation.getMatchTime();
        double nextTargetTime = 0.0;
        boolean isRed = alliance.equals(DriverStation.Alliance.Red);

        // 1. Calculate the target time for the countdown
        // "R" Red and "B" Blue share the exact same schedule
        if ((gameData.contains("R") && isRed) || (gameData.contains("B") && !isRed)) {
            if (matchTime > 127) nextTargetTime = 127;
            else if (matchTime > 108) nextTargetTime = 108;
            else if (matchTime > 77) nextTargetTime = 77;
            else if (matchTime > 58) nextTargetTime = 58;
        } 
        // "R" Blue and "B" Red share the exact same schedule
        else if ((gameData.contains("R") && !isRed) || (gameData.contains("B") && isRed)) {
            if (matchTime > 102) nextTargetTime = 102;
            else if (matchTime > 83) nextTargetTime = 83;
            else if (matchTime > 52) nextTargetTime = 52;
            else if (matchTime > 33) nextTargetTime = 33;
        }

        // Sets the live countdown (prevents dropping below 0)
        double timeUntilSwitch = Math.max(0, matchTime - nextTargetTime);
        SmartDashboard.putNumber("Robot/TimeUntilSwitchSec", timeUntilSwitch);

        // 2. Main State Machine
        if (gameData.contains("R")) {
            if (isRed) {
                if (matchTime > 127) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 130) {
                        switchingRed = true;
                    } else if (matchTime < 137) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 108) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 111) {
                        switchingGreen = true;
                    } else if (matchTime < 118) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 77) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 80) {
                        switchingRed = true;
                    } else if (matchTime < 87) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 58) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 61) {
                        switchingGreen = true;
                    } else if (matchTime < 68) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 30) {
                    switching = false;
                    switchingRed = false;
                    switchingGreen = false;
                    exampleColor = blackColor;
                    setState(RobotState.ACTIVE);
                } else {
                    setState(RobotState.ACTIVE);
                }
            } else { // Blue Alliance
                if (matchTime > 130) {
                    setState(RobotState.ACTIVE);
                    switching = false;
                } else if (matchTime > 102) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 105) {
                        switchingRed = true;
                    } else if (matchTime < 112) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 83) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 86) {
                        switchingGreen = true;
                    } else if (matchTime < 93) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 52) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 55) {
                        switchingRed = true;
                    } else if (matchTime < 62) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 33) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 36) {
                        switchingGreen = true;
                    } else if (matchTime < 43) { 
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else {
                    switching = false;
                    switchingRed = false;
                    switchingGreen = false;
                    exampleColor = blackColor;
                    setState(RobotState.ACTIVE);
                }
            }
        } else if (gameData.contains("B")) {
            if (!isRed) { // Blue Alliance
                if (matchTime > 127) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 130) {
                        switchingRed = true;
                    } else if (matchTime < 137) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 108) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 111) {
                        switchingGreen = true;
                    } else if (matchTime < 118) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 77) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 80) {
                        switchingRed = true;
                    } else if (matchTime < 87) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 58) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 61) {
                        switchingGreen = true;
                    } else if (matchTime < 68) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 30) {
                    switching = false;
                    switchingRed = false;
                    switchingGreen = false;
                    exampleColor = blackColor;
                    setState(RobotState.ACTIVE);
                } else {
                    setState(RobotState.ACTIVE);
                }
            } else { // Red Alliance
                if (matchTime > 130) {
                    setState(RobotState.ACTIVE);
                    switching = false;
                } else if (matchTime > 102) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 105) {
                        switchingRed = true;
                    } else if (matchTime < 112) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 83) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 86) {
                        switchingGreen = true;
                    } else if (matchTime < 93) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 52) {
                    setState(RobotState.ACTIVE);
                    if (matchTime < 55) {
                        switchingRed = true;
                    } else if (matchTime < 62) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else if (matchTime > 33) {
                    setState(RobotState.INACTIVE);
                    if (matchTime < 36) {
                        switchingGreen = true;
                    } else if (matchTime < 43) {
                        switching = true;
                    } else {
                        switching = false;
                        switchingRed = false;
                        switchingGreen = false;
                        exampleColor = blackColor;
                    }
                } else {
                    switching = false;
                    switchingRed = false;
                    switchingGreen = false;
                    exampleColor = blackColor;
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
        if (pose.getX() < 5.4) {
            currentZone = alliance.equals(Alliance.Blue) ? FieldZone.ALLIANCE : FieldZone.OPPONENT;
            return currentZone;
        } else if (pose.getX() > 11.0) {
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

    public boolean underTrench() {
        double xPose = pose.getX();
        double yPose = pose.getY();
        if (xPose > 3.7 && xPose < 5.3 && yPose > 6.5 && yPose < 8.3) {
            return true;
        } else if (xPose > 3.7 && xPose < 5.3 && yPose < 1.8 && yPose > 0) {
            return true;
        } else if (xPose > 10.9 && xPose < 12.8 && yPose > 6.5 && yPose < 8.3) {
            return true;
        } else if (xPose > 10.9 && xPose < 12.8 && yPose < 1.8 && yPose > 0) {
            return true;
        } else {
            return false;
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

    public boolean isInAlliance() {
        return checkZone() == FieldZone.ALLIANCE;
    }

}

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
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
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
 * The full shot-computation chain runs exactly once per loop, from {@link #periodic()} via
 * the private {@code computeAimParams()}, and the result is cached. {@link #getAimParams()}
 * — the public entry point used by {@code Turret.track}, {@code Shooter.shoot}, and telemetry
 * — is a plain field read of that cache, not a fresh computation. (Before 2026-09-09 this ran
 * on every call instead; see {@link #getAimParams()}'s Javadoc for why that changed.)
 * <pre>
 *   periodic() [once per loop]
 *     → computeAimParams()
 *         → LeadCompensator.computeLeadTarget(hub, turretPose, fieldVelocity, m_tofAim)
 *             → m_tofAim.update(virtualTarget, turretPose, kZero)   [15-iter convergence]
 *             → AimParams { yaw, pitch, output, tof }
 *         → m_tofAim.update(virtualTarget, turretPose, kZero)       [final params]
 *         → cached in m_cachedAimParams
 *   getAimParams() → m_cachedAimParams  (used by Turret.track and Shooter.shoot)
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
    /**
     * Must be declared before {@link #instance} below. Java runs static field initializers in
     * textual/declaration order — {@code instance}'s initializer calls {@code new
     * RobotStateMachine()} immediately, which runs every instance field initializer in this
     * class, including {@code m_tofAim = new ToFAim(..., kScoringConstraints)}. If this field
     * were declared <em>after</em> {@code instance} (as it was until 2026-09-09), that
     * constructor call would run before this field's own initializer had executed, so
     * {@code kScoringConstraints} would still be {@code null} at the moment {@code m_tofAim}
     * captured it — permanently, since {@code ToFAim}'s constructor just does a plain field
     * assignment, not a live reference. The bug was latent for a long time because nothing
     * called {@link #getAimParams()} reliably until {@code OnboardLogger.logAll()} was wired
     * into {@code Robot.robotPeriodic()} the same day — once every loop started reaching
     * {@code computeAimParams()} unconditionally, the resulting
     * {@code NullPointerException} in {@code AimConstraints.check()} crashed the robot
     * immediately on the very first loop. See {@code plans/review_plan.md} for the fix.
     */
    private static final AimConstraints kScoringConstraints = new AimConstraints(
        Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(80), 100.0);

    private static final RobotStateMachine instance = new RobotStateMachine();

    private RobotState state = RobotState.ACTIVE;
    private String gameData = "";
    private boolean gotData = false;

    private boolean switching = false;
    private boolean switchingRed = false;
    private boolean switchingGreen = false;
    private Color exampleColor;
    private Color whiteColor = new Color(237, 237, 237);
    private Color blackColor = new Color(49, 49, 49);
    private Color redColor = new Color(191, 0, 0);
    private Color greenColor = new Color(0, 191, 0);

    private Pose2d turretPose = new Pose2d();
    private Pose3d m_lastLeadTarget = new Pose3d();
    /** Cache for {@link #getAimParams()} — refreshed once per loop in {@link #periodic()}. */
    private AimParams m_cachedAimParams = AimParams.impossible();

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

    /** Returns the shared {@link Shooter} subsystem owned by this state machine. */
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
    /**
     * Returns the cached result of the most recent aiming pipeline evaluation.
     *
     * <p>Found and fixed 2026-09-09 (see {@code plans/review_plan.md} R2-B1/B2): this used to
     * run the full pipeline — {@link LeadCompensator} (up to 5 iterations) plus a final
     * {@link ToFAim#update}, allocating {@link Pose3d}/{@link Translation2d}/{@link AimParams}
     * throughout — on <em>every call</em>. While shooting, that meant roughly 10 identical
     * evaluations per 20 ms loop: twice from {@code Turret.track()}, once from
     * {@code Shooter.shoot()}, and (once {@link OnboardLogger#logAll()} was wired up the same
     * day) seven more from {@link frc.robot.aiming.AimParams#setupLogging}'s telemetry
     * suppliers. All ten computed the exact same numeric result.
     *
     * <p>The pipeline now runs exactly once per loop, in {@link #periodic()} right after
     * {@link #turretPose} is recomputed — see {@link #computeAimParams()}. This does
     * <b>not</b> introduce new staleness: {@code turretPose} was already refreshed only once
     * per loop, in {@code periodic()}, which itself runs after {@code CommandScheduler.run()}
     * in {@code Robot.robotPeriodic()} — so commands executing during the scheduler pass were
     * already reading the previous loop's {@code turretPose} before this change. Caching
     * merely stops recomputing the same numbers from the same stale input; it does not change
     * what input is used.
     *
     * @return shot parameters; status is {@link frc.robot.aiming.AimParams.AimStatus#Impossible}
     *         when no target is available, the distance is out of range, or the required angle
     *         violates {@code kScoringConstraints}
     */
    public AimParams getAimParams() {
        return m_cachedAimParams;
    }

    /**
     * Runs the full aiming pipeline. Called exactly once per loop from {@link #periodic()};
     * all other code should call {@link #getAimParams()} instead, which reads the cached
     * result. See {@link #getAimParams()}'s Javadoc for why this split exists.
     */
    private AimParams computeAimParams() {
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

    /**
     * Returns {@code true} when the shooter flywheel is at the target speed for the current
     * aim parameters. Convenience wrapper used by button bindings and SmartDashboard telemetry.
     *
     * <p>Calls {@link Shooter#isTracked(AimParams)} directly against the cached
     * {@link #getAimParams()} result, rather than going through {@link Shooter#tracked}
     * (which allocates a {@link edu.wpi.first.wpilibj2.command.button.Trigger} and a wrapping
     * lambda). Since this method itself runs every loop via {@link StateManager#shootReady},
     * that allocation would otherwise happen every loop too. Changed 2026-09-09 — see
     * {@code plans/review_plan.md} R2-B3.
     */
    public boolean isShootReady() {
        return m_Shooter.isTracked(getAimParams());
    }

    /** Returns the driver's command controller (port 0). */
    public CommandXboxController getDriver() {
        return joystick;
    }

    /** Returns the gunner's Xbox controller (port 1). */
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
        // Must run after turretPose above — computeAimParams() reads it. Runs exactly once per
        // loop here; see getAimParams()'s Javadoc for why this is cached rather than computed
        // on every call.
        m_cachedAimParams = computeAimParams();

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

    /**
     * Advances the LED blink pattern by one step, called at 10 Hz from {@link #periodic()}.
     *
     * <p>Three blink modes are supported:
     * <ul>
     *   <li>{@code switchingRed} — alternates between red and black to signal this alliance's
     *       scoring window opening (drive team should start shooting).
     *   <li>{@code switchingGreen} — alternates between green and black to signal a scoring
     *       window closing (drive team should stop shooting).
     *   <li>{@code switching} — alternates between white and black as a neutral countdown flash
     *       during the 7-second transition buffer before the window boundary.
     * </ul>
     * Only one mode is active at a time; {@link #getState()} sets the flags and clears all three
     * when the transition buffer expires.
     */
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

    /**
     * Updates {@link #Tag_POSE2D} and {@link #HubPose} for the current alliance.
     *
     * <p>AprilTag IDs used as the scoring target:
     * <ul>
     *   <li>Tag 10 — Red alliance hub (field center, red side)
     *   <li>Tag 20 — Blue alliance hub (field center, blue side)
     * </ul>
     *
     * <p>The hub's physical center is 0.5842 m behind (in the tag's facing direction) the
     * AprilTag face, because the tag is mounted on the rim of the hub, not at its center.
     * {@code HubPose} compensates for this offset and represents the actual ball-entry point
     * used by the aiming pipeline.
     */
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

    /** Returns the hub center pose (0.5842 m behind the AprilTag face). */
    public Pose2d getHubPose() {
        return HubPose;
    }

    /**
     * Returns the robot's current field-relative chassis speeds, or {@code null} if the
     * drivetrain has not yet been bound via {@link #bindDrivetrain(CommandSwerveDrivetrain)}.
     * Callers must null-check the result.
     */
    public ChassisSpeeds getFieldSpeeds() {
        if (drivetrain == null) {
            return null;
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), pose.getRotation());
    }

    /**
     * Returns the robot-relative chassis speeds from the drivetrain's odometry state.
     * Returns zero speeds if the drivetrain has not been bound yet.
     */
    public ChassisSpeeds getChassisSpeeds() {
        if (drivetrain == null) return new ChassisSpeeds();
        return drivetrain.getState().Speeds;
    }

    /**
     * Resets the vision estimator's internal pose to {@code pose}. Use after a known
     * field-relative position is established (e.g., after placing against a wall at auto start).
     */
    public void resetVisionPose(Pose2d pose) {
        m_vision.resetVisionPose(pose);
    }

    /**
     * Binds the drivetrain subsystem so field-relative speeds can be fetched for lead
     * compensation. Must be called from {@link RobotContainer} before teleop begins.
     */
    public void bindDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    /** Returns the straight-line distance from the robot to the hub center (meters). */
    public double distToTag() {
        return pose.getTranslation().getDistance(HubPose.getTranslation());
    }

    /**
     * Returns {@code true} when the robot's shooter face is pointed within 20° of the hub.
     *
     * <p>The shooter is mounted on the <em>back</em> of the robot, so the facing direction is
     * the robot heading rotated by π (180°). Subtracting π from the robot heading converts
     * "robot forward" to "shooter forward" before computing the angular error to the hub.
     */
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
     * Drives the LED color state machine based on match time and game-specific data, then
     * returns the current {@link RobotState}.
     *
     * <p><b>Warning — side effects:</b> This method calls {@link #setState(RobotState)} (which
     * in turn calls {@link #refreshPoseFromVision()} and {@link #update(RobotState)}) and
     * mutates {@code switching}, {@code switchingRed}, {@code switchingGreen}, and
     * {@code exampleColor}. It also writes {@code Robot/TimeUntilSwitchSec} to SmartDashboard.
     * Do not call this method when you only want to read the state — call the field {@code state}
     * directly via {@link #isActive()} instead.
     *
     * <h2>LED / Scoring-Window Schedule</h2>
     * The FMS game-specific message encodes which hub color is active. When the first character
     * is {@code 'R'} or {@code 'B'}, two interleaved schedules run — one for the alliance whose
     * color was announced, one for the opposing alliance. At each schedule boundary a 3-second
     * flash sequence fires ({@code switchingRed} → red blink, {@code switchingGreen} → green
     * blink, {@code switching} → white blink) to signal the drive team that the scoring window
     * is changing. The flash sequences are coordinated through {@link #newPostedValue()}, which
     * is called at 10 Hz from {@link #periodic()}.
     *
     * @return current {@link RobotState} after applying all state transitions
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
     * Requests a transition to the specified state. No-ops if already in that state.
     * Refreshes the cached pose from vision before applying the transition so any LED or
     * aiming logic that runs immediately after has an up-to-date position.
     *
     * @param next the desired state
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

    // Removed 2026-09-09 (plans/review_plan.md R4-C7): underTrench() had zero callers.
    // Its Javadoc claimed "used by the flywheel to limit hood angle while passing under the
    // trench bar," but Flywheel.java was deleted in Stage 5 and nothing replaced that caller.
    // Preserving the field geometry it encoded, in case trench-limiting behavior is rebuilt
    // for the current Shooter/hood mechanism — field coordinates in meters, origin at the
    // blue alliance wall corner:
    //   Blue trench top:    x in [3.7, 5.3], y in [6.5, 8.3]
    //   Blue trench bottom: x in [3.7, 5.3], y in [0, 1.8]
    //   Red trench top:     x in [10.9, 12.8], y in [6.5, 8.3]
    //   Red trench bottom:  x in [10.9, 12.8], y in [0, 1.8]

    /** Returns the cached game-specific message string (may be empty before FMS connects). */
    public String getGameData() {
        return gameData;
    }

    /**
     * Stores the game-specific message and marks it as received so {@link #hasData()} returns
     * {@code true}. Once data is received, {@link Robot#teleopPeriodic()} stops polling.
     */
    public void setGameData(String data) {
        gameData = data;
        gotData = true;
    }

    /**
     * Returns {@code true} once the game-specific message has been received from the FMS
     * or Driver Station. Used to gate the one-time alliance/state setup in teleop.
     */
    public boolean hasData() {
        return gotData;
    }

    /**
     * Returns the current alliance from the Driver Station, defaulting to Blue when the
     * alliance is not yet known (e.g., during simulation or before FMS connect).
     */
    public Alliance getAlliance() {
        return DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
    }

    /**
     * Whether the robot is allowed to shoot.
     * ACTIVE means the scoring window is open for this alliance; INACTIVE means it is not.
     */
    public enum RobotState {
        /** Scoring window is open — robot may shoot. */
        ACTIVE,
        /** Scoring window is closed — robot should not shoot. */
        INACTIVE
    }

    /** Describes where the robot is on the field relative to alliance boundaries. */
    public enum FieldZone {
        /** Robot is in its own alliance's protected zone (x < 5.4 m for Blue, x > 11.0 m for Red). */
        ALLIANCE,
        /** Robot is in the neutral zone near the top of the field (y > 4.2 m). */
        NEUTRAL_TOP,
        /** Robot is in the neutral zone near the center of the field (3.8 m ≤ y ≤ 4.2 m). */
        NEUTRAL_CENTER,
        /** Robot is in the neutral zone near the bottom of the field (y < 3.8 m). */
        NEUTRAL_BOTTOM,
        /** Robot is in the opponent's protected zone. */
        OPPONENT
    }

    /** Returns {@code true} when the current state is {@link RobotState#ACTIVE}. */
    public boolean isActive() {
        return getState() == RobotState.ACTIVE;
    }

    /** Returns {@code true} when the robot is in its own alliance's zone. */
    public boolean isInAlliance() {
        return checkZone() == FieldZone.ALLIANCE;
    }

}

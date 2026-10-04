package frc.robot;

import static edu.wpi.first.units.Units.Meter;

import java.util.Optional;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.turret.Hood;
import frc.robot.utility.shooting.ShotCalibration;
import frc.robot.utility.shooting.ShotPlanner;
import frc.robot.utility.shooting.ShotSafety;
import frc.robot.utility.shooting.ShotSample;
import frc.robot.utility.shooting.ShotSettings;
import frc.robot.utility.shooting.ShotSolution;
import frc.robot.utility.shooting.ShotSolver;
import frc.robot.utility.shooting.ShotTable;

/**
 * Singleton state machine that tracks robot state, pose, and field zone.
 */
public final class RobotStateMachine {
    private static RobotStateMachine instance;

    private RobotState state = RobotState.ACTIVE;
    private String gameData = "";
    private boolean gotData = false;

    public double shooterSpeed;
    public double reqShooterSpeed;

    public boolean ductTapeCorrection = false;

    private boolean switching = false;
    private boolean switchingRed = false;
    private boolean switchingGreen = false;
    private Color exampleColor;
    private Color whiteColor = new Color(237, 237, 237);
    private Color blackColor = new Color(49, 49, 49);
    private Color redColor = new Color(191, 0, 0);
    private Color greenColor = new Color(0, 191, 0);

    private Pose2d turretPose = new Pose2d();

    private Vision m_vision;
    private Flywheel m_Flywheel;
    private Turret m_Turret;

    public static Pose3d Tag_POSE2D;

    public static Pose2d HubPose;

    private Pose2d targetPose = new Pose2d();

    private final Field2d aimTargetField = new Field2d();

    private final Hood m_Hood;
    private final ShotSolver shotSolver;
    private final ShotPlanner shotPlanner;
    private final ShotCalibration shotCalibration;
    private String shotTableError = "";

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

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final XboxController m_gunner = new XboxController(1);

    private RobotStateMachine() {
        checkAlliance();

        exampleColor = whiteColor;

        m_Flywheel = new Flywheel(this);
        m_Turret = new Turret(this);
        m_Hood = new Hood();
        ShotSolver configuredSolver;
        try {
            configuredSolver = new ShotSolver(ShotTable.samples(), TurretConstants.kHoodMinPositionRotations,
                    TurretConstants.kHoodMaxPositionRotations);
        } catch (IllegalArgumentException error) {
            shotTableError = error.getMessage();
            DriverStation.reportError("Invalid shot table: " + shotTableError, false);
            configuredSolver = new ShotSolver(new ShotSample[0],
                    TurretConstants.kHoodMinPositionRotations, TurretConstants.kHoodMaxPositionRotations);
        }
        shotSolver = configuredSolver;
        shotPlanner = new ShotPlanner(shotSolver);
        shotCalibration = new ShotCalibration(this);

        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
        SmartDashboard.putData("Aim Target Poses", aimTargetField);
    }

    public Flywheel getFlywheel() {
        return m_Flywheel;
    }

    public Hood getHood() { return m_Hood; }
    public ShotCalibration getShotCalibration() { return shotCalibration; }

    public CommandXboxController getDriver() {
        return joystick;
    }

    public XboxController getGunner() {
        return m_gunner;
    }

    public double getConvertedTurretPosition() {
        return m_Turret.getConvertedTurretPosition();
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
        shotCalibration.readInputs(DriverStation.isEnabled() && !DriverStation.isAutonomous());
        reqShooterSpeed = m_Flywheel.getReqSpeed();
        shooterSpeed = m_Flywheel.getSpeed();
        SmartDashboard.putBoolean("Driver Connected", joystick.isConnected());
        SmartDashboard.putBoolean("Gunner Connected", m_gunner.isConnected());
        SmartDashboard.putBoolean("ductTapeCorrections", ductTapeCorrection);
        gameData = DriverStation.getGameSpecificMessage();
        alliance = getAlliance();
        checkAlliance();
        refreshPoseFromVision();
        currentZone = checkZone();
        posePublisher.set(pose);

        // 1. Get the turret's base position on the field
        Pose2d turretBase = pose.transformBy(TurretConstants.ROBOT_TO_TURRET_BASE);

        // 2. Combine the robot's heading and turret's relative rotation
        Rotation2d finalRotation = turretBase.getRotation().plus(
                Rotation2d.fromDegrees(m_Turret.getConvertedTurretPosition()));

        // 3. Create the final pose using the base translation and the summed rotation
        turretPose = new Pose2d(turretBase.getTranslation(), finalRotation);

        // turretPose = pose.transformBy(TurretConstants.ROBOT_TO_TURRET_BASE)
        // .plus(new Transform2d(0, 0,
        // Rotation2d.fromDegrees(m_Turret.getConvertedTurretPosition())));

        turretPosePublisher.set(turretPose);
        SmartDashboard.putString("Yall we're switching", exampleColor.toHexString());
        newPostedValue();
        SmartDashboard.putString("RobotState", state.toString());
        SmartDashboard.putString("FieldZone", currentZone.toString());
        SmartDashboard.putBoolean("IsActive", isActive());
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        SmartDashboard.putNumber("distToTag2", distToTag());
        SmartDashboard.putBoolean("isFacing", isFacingHub());
        updateTargetPose();
        applyShotSetpoints();

        aimTargetField.getObject("Hub").setPose(HubPose);
        aimTargetField.getObject("Motion Compensated Hub").setPose(targetPose);

        SmartDashboard.putNumber("Hub X (m)", HubPose.getX());
        SmartDashboard.putNumber("Hub Y (m)", HubPose.getY());
        SmartDashboard.putNumber("Hub Heading (deg)", HubPose.getRotation().getDegrees());
        SmartDashboard.putNumber("Motion Compensated Hub X (m)", targetPose.getX());
        SmartDashboard.putNumber("Motion Compensated Hub Y (m)", targetPose.getY());
        SmartDashboard.putNumber("Motion Compensated Hub Heading (deg)",
                targetPose.getRotation().getDegrees());
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
        // double allianceMulti = 1;
        if (getAlliance() == Alliance.Red) {
            Tag_POSE2D = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(10).get();
        } else {
            Tag_POSE2D = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get();
            // allianceMulti = -1;
        }
        HubPose = Tag_POSE2D.toPose2d().transformBy(
                new Transform2d(Distance.ofRelativeUnits(-0.5842, Meter),
                        Distance.ofBaseUnits(0, Meter),
                        new Rotation2d()));
    }

    public Pose2d getHubPose() {
        return HubPose;
    }

    public Pose2d getTurretPose() {
        return turretPose;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    /**
     * Returns the current shared target, velocity, and flywheel solution.
     *
     * @return the latest shot solution
     */
    public ShotSolution getShotSolution() {
        return shotPlanner.getSolution();
    }

    /** Compute one snapshot per cycle; getters and commands never recompute it. */
    private void updateTargetPose() {
        ChassisSpeeds speeds = getFieldSpeeds();
        targetPose = HubPose;
        double distance = turretPose.getTranslation().getDistance(HubPose.getTranslation());
        ShotSolution solution;
        if (!DriverStation.isEnabled()) {
            solution = shotPlanner.invalidate(HubPose, distance, "Disabled");
        } else if (speeds == null) {
            solution = shotPlanner.invalidate(HubPose, distance, "Drivetrain unavailable");
        } else if (!isInAlliance() || underTrench()) {
            solution = shotPlanner.invalidate(HubPose, distance, "Passing/trench profile not calibrated");
        } else if (!shotCalibration.isEnabled() && DriverStation.isTest()) {
            solution = shotPlanner.invalidate(HubPose, distance, "Test mode: enable calibration to tune shots");
        } else if (!shotCalibration.isEnabled() && !isActive()) {
            solution = shotPlanner.invalidate(HubPose, distance, "Hub inactive");
        } else {
            Translation2d velocity = getTurretFieldVelocity(speeds);
            boolean stationary = ShotSafety.isStationary(velocity.getX(), velocity.getY(), speeds.omegaRadiansPerSecond);
            if (shotCalibration.isEnabled()) {
                Optional<ShotSettings> manual = shotCalibration.manualSettings();
                if (!stationary) {
                    solution = shotPlanner.invalidate(HubPose, distance, "Stop robot for manual calibration");
                } else if (manual.isEmpty()) {
                    solution = shotPlanner.invalidate(HubPose, distance, "Invalid calibration inputs");
                } else {
                    solution = shotPlanner.manual(HubPose, distance, manual.get());
                }
            } else if (!shotTableError.isEmpty()) {
                solution = shotPlanner.invalidate(HubPose, distance, "Invalid table: " + shotTableError);
            } else if (!shotCalibration.isMotionEnabled() && !stationary) {
                solution = shotPlanner.invalidate(HubPose, distance, "Motion compensation disabled; stop robot");
            } else if (!Double.isFinite(shotCalibration.getFlywheelTrimRps())
                    || (!stationary && shotCalibration.getFlywheelTrimRps() != 0)) {
                solution = shotPlanner.invalidate(HubPose, distance, "Moving shots require zero flywheel trim");
            } else {
                solution = shotPlanner.update(turretPose, HubPose, velocity, shotCalibration.isMotionEnabled());
                if (solution.isValid() && solution.getFlywheelSpeed() + shotCalibration.getFlywheelTrimRps() <= 0) {
                    solution = shotPlanner.invalidate(HubPose, distance, "Flywheel trim produces nonpositive RPS");
                }
            }
        }
        targetPose = solution.getTargetPose();
        targetPosePublisher.set(targetPose);
        SmartDashboard.putBoolean("Shot/Valid", solution.isValid());
        SmartDashboard.putString("Shot/Status", solution.status());
        SmartDashboard.putNumber("ShotDistance", solution.getDistance());
        SmartDashboard.putNumber("EffectiveShotDistance", solution.getEffectiveDistance());
        SmartDashboard.putNumber("ShotTOF", solution.getTimeOfFlight());
        SmartDashboard.putNumber("ShotVelocity", solution.getFlywheelSpeed());
        SmartDashboard.putNumber("Shot/Hood Rotations", solution.getHoodRotations());
        SmartDashboard.putNumber("RadialVelocity", solution.getRadialVelocity());
        SmartDashboard.putNumber("TurretVelX", solution.getTurretVelocityX());
        SmartDashboard.putNumber("TurretVelY", solution.getTurretVelocityY());
    }

    private void applyShotSetpoints() {
        // Characterization commands own motor output in test mode unless tuning is enabled.
        if (DriverStation.isEnabled() && DriverStation.isTest() && !shotCalibration.isEnabled()) { return; }
        ShotSolution solution = getShotSolution();
        if (solution.isValid()) {
            double trim = shotCalibration.isEnabled() ? 0 : shotCalibration.getFlywheelTrimRps();
            m_Flywheel.setSpeed(solution.getFlywheelSpeed() + trim);
            m_Hood.setPositionRotations(solution.getHoodRotations());
        } else {
            m_Flywheel.stopMotor();
            m_Hood.stop();
        }
    }

    /** Current physical alignment, independent of whether an aiming command is scheduled. */
    public boolean isTurretAligned() {
        Translation2d vector = targetPose.getTranslation().minus(turretPose.getTranslation());
        return vector.getNorm() > 1e-9 && m_Turret.isHealthy() && !m_Turret.isHoming()
                && Math.abs(ShotSafety.alignmentErrorDegrees(vector.getAngle().getDegrees(),
                        turretPose.getRotation().getDegrees())) <= 1.5
                && Math.abs(m_Turret.getSpeed()) <= 0.1;
    }

    /** All shooting sequences use this gate and explicitly stop when it becomes false. */
    public boolean canFeedShot() {
        return ShotSafety.canFeed(DriverStation.isEnabled(), shotCalibration.isEnabled() || isActive(),
                getShotSolution().isValid(), isTurretAligned(), m_Flywheel.isUpToSpeed(), m_Hood.isAtPosition());
    }

    /** Compatibility lookup; no extrapolation or legacy TOF table. */
    public double getTOF(double distanceMeters) {
        return shotSolver.solve(distanceMeters).map(ShotSettings::tofSeconds).orElse(Double.NaN);
    }

    public Turret getTurret() {
        return m_Turret;
    }

    public ChassisSpeeds getFieldSpeeds() {
        if (drivetrain == null) {
            return null;
        }
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), pose.getRotation());
    }

    /**
     * Calculates the field-relative velocity of the turret base, including the
     * robot's rotational contribution at the turret offset.
     *
     * @param fieldSpeeds field-relative chassis speeds
     * @return field-relative turret-base velocity
     */
    public Translation2d getTurretFieldVelocity(ChassisSpeeds fieldSpeeds) {
        Translation2d turretOffset = TurretConstants.ROBOT_TO_TURRET_BASE.getTranslation()
                .rotateBy(pose.getRotation());
        return new Translation2d(
                fieldSpeeds.vxMetersPerSecond - fieldSpeeds.omegaRadiansPerSecond * turretOffset.getY(),
                fieldSpeeds.vyMetersPerSecond + fieldSpeeds.omegaRadiansPerSecond * turretOffset.getX());
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

    // public void getVisionEst(String name) {
    // m_vision.getEstPoses(name);
    // }

    public void bindDrivetrain(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public double distToTag() {
        return pose.getTranslation().getDistance(HubPose.getTranslation());
    }

    public boolean isFarEnough() {
        return distToTag() > 3.3;
    }

    public boolean isUpToSpeed() {
        return m_Flywheel.isUpToSpeed();
    }

    public boolean isFacingHub() {
        double dx = targetPose.getX() - pose.getX();
        double dy = targetPose.getY() - pose.getY();
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
            if (matchTime > 127)
                nextTargetTime = 127;
            else if (matchTime > 108)
                nextTargetTime = 108;
            else if (matchTime > 77)
                nextTargetTime = 77;
            else if (matchTime > 58)
                nextTargetTime = 58;
        }
        // "R" Blue and "B" Red share the exact same schedule
        else if ((gameData.contains("R") && !isRed) || (gameData.contains("B") && isRed)) {
            if (matchTime > 102)
                nextTargetTime = 102;
            else if (matchTime > 83)
                nextTargetTime = 83;
            else if (matchTime > 52)
                nextTargetTime = 52;
            else if (matchTime > 33)
                nextTargetTime = 33;
        }

        // Sets the live countdown (prevents dropping below 0)
        double timeUntilSwitch = Math.max(0, matchTime - nextTargetTime);
        SmartDashboard.putNumber("Time Until Switch", timeUntilSwitch);

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

    public double getPoseTime() {
        return m_vision.getPoseTime();
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
            ductTapeCorrection = true;
            return true;
        } else if (xPose > 3.7 && xPose < 5.3 && yPose < 1.8 && yPose > 0) {
            ductTapeCorrection = false;
            return true;
        } else if (xPose > 10.9 && xPose < 12.8 && yPose > 6.5 && yPose < 8.3) {
            ductTapeCorrection = true;
            return true;
        } else if (xPose > 10.9 && xPose < 12.8 && yPose < 1.8 && yPose > 0) {
            ductTapeCorrection = false;
            return true;
        } else {
            ductTapeCorrection = false;
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

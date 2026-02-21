// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.derive;

import java.io.BufferedWriter;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.OpenOption;
import java.nio.file.StandardOpenOption;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import javax.crypto.ShortBufferException;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlignTurretToHub;
import frc.robot.commands.MoveTurret;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootingSequence;
import frc.robot.generated.TunerConstants2;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LedCANdle;
import frc.robot.subsystems.turret.Flywheel;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.limelight.LimelightHelpers;
import frc.robot.subsystems.vision.limelight.LimelightIO;
import frc.robot.subsystems.vision.photonvision.PhotonVisionIO;
import frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO;
import frc.robot.utility.RangeFinder;
import frc.robot.utility.ShooterValuesSenable;

/**
 * Central robot wiring for subsystems, commands, and operator bindings.
 */
public class RobotContainer {
        @SuppressWarnings("unused")
        private double speedModify = 1;
        private double MaxSpeed = TunerConstants2.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                       // speed
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second
                                                                                          // max angular velocity

        /* Setting up bindings for necessary control of the swerve drive platform */
        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive
                                                                                 // motors
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final Telemetry logger = new Telemetry(MaxSpeed);

        private final CommandXboxController joystick = new CommandXboxController(0);
        private final CommandPS4Controller pranav = new CommandPS4Controller(0);

        private final CommandXboxController joystick2 = new CommandXboxController(1);

        public final CommandSwerveDrivetrain drivetrain = TunerConstants2.createDrivetrain();

        private LedCANdle m_candle = new LedCANdle();

        private final Hopper hopper = new Hopper();

        private final SendableChooser<Command> autoChooser;

        private final Flywheel m_flywheel = new Flywheel();

        private final Turret m_turret = new Turret();

        private final Intake m_intake = new Intake();

        private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
        private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(25).get();

        private final File logDir;
        private final File logFile;
        private BufferedWriter writer = null;
        private final SendableChooser<Boolean> closeLogSendable = new SendableChooser<Boolean>();
        private final SendableChooser<Boolean> shooterSendableChooser = new SendableChooser<Boolean>();
        private final Sendable shooterSendable = new ShooterValuesSenable();

        // private final Feeder m_feeder = new Feeder();
        // private final Intake m_intake = new Intake();
        // private final Shooter m_shooter = new Shooter();

        // Vision
        PhotonVisionIO photonVisionIO;
        private final Vision m_vision;

        /**
         * Creates the container, initializes logging, chooser options, and vision.
         */
        public RobotContainer() {
                NamedCommands.registerCommand("IntakeFuel", new RunIntake(m_intake, -3));

                logDir = new File("log");
                logDir.mkdirs();
                logFile = new File(logDir, "shootFile.json");
                logFile.setWritable(true);
                closeLogSendable.setDefaultOption("false", false);
                closeLogSendable.addOption("true", true);
                SmartDashboard.putData("Close Buffer", closeLogSendable);

                SmartDashboard.putData("Shooter Values", shooterSendable);

                try {
                        writer = Files.newBufferedWriter(logFile.toPath(), StandardOpenOption.CREATE,
                                        StandardOpenOption.WRITE);
                } catch (IOException e) {
                        System.err.println("Could not open the JSON log file.");
                        e.printStackTrace();
                }

                autoChooser = AutoBuilder.buildAutoChooser("testAuto");
                SmartDashboard.putData("Auto Chooser", autoChooser);
                configureBindings();
                CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
                switch (RobotConstants.currentMode) {
                        case REAL:
                                PhotonVisionIO m_photonVisionIO = new PhotonVisionIO("Thrifty_cam_2", false,
                                                new Translation3d(0.254, 0.254, 0.2032),
                                                new Rotation3d(0, Math.toRadians(62), Math.toRadians(42)));
                                PhotonVisionIO m_photonVisionIO2 = new PhotonVisionIO("Thrifty_cam_1", false,
                                                new Translation3d(0.254, 0.254, 0.2032),
                                                new Rotation3d(0, Math.toRadians(62), Math.toRadians(42)));
                                LimelightIO m_ll = new LimelightIO("limelight-gcd", true, drivetrain.rotationSupplier(),
                                                drivetrain.getAngularVel(),
                                                true);
                                LimelightIO m_ll2 = new LimelightIO("limelight-gcc", true,
                                                drivetrain.rotationSupplier(),
                                                drivetrain.getAngularVel(),
                                                true);
                                m_vision = new Vision(
                                                drivetrain.rotationSupplier(),
                                                drivetrain.modulePositionsSupplier(),
                                                drivetrain.poseSupplier(),
                                                m_photonVisionIO,
                                                m_photonVisionIO2,
                                                m_ll,
                                                m_ll2);
                                break;
                        case SIM:
                                // TODO: Add Real Camera Constants to use here
                                SimCameraProperties cameraProp = new SimCameraProperties();
                                cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
                                // Approximate detection noise with average and standard deviation error in
                                // pixels.
                                cameraProp.setCalibError(0.25, 0.08);
                                cameraProp.setFPS(60);
                                cameraProp.setAvgLatencyMs(35);
                                cameraProp.setLatencyStdDevMs(5);
                                PhotonVisionSimIO camSim = new PhotonVisionSimIO("photonvision", false, cameraProp,
                                                new Translation3d(0.1, 0, 0.5),
                                                new Rotation3d(0, Math.toRadians(-15), 0));
                                m_vision = new Vision(
                                                drivetrain.rotationSupplier(),
                                                drivetrain.modulePositionsSupplier(),
                                                drivetrain.poseSupplier(),
                                                camSim);
                                break;
                        default:
                                m_vision = new Vision();
                                break;
                }
                robotStateMachine.bindVision(m_vision);
                robotStateMachine.bindChassisSpeedsSupplier(() -> drivetrain.getKinematics().toChassisSpeeds(
                                drivetrain.getModule(0).getCurrentState(), drivetrain.getModule(1).getCurrentState(),
                                drivetrain.getModule(2).getCurrentState(),
                                drivetrain.getModule(3).getCurrentState()));
                setRobotOrientation();
        }

        /**
         * Configures controller bindings and default commands.
         */
        private void configureBindings() {
                // Note that X is defined as forward according to WPILib convention,
                // and Y is defined as to the left according to WPILib convention.
        // @formatter:off
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) * MaxSpeed * m_flywheel.speedModifier) // Drive forward with negative Y (forward)
                                .withVelocityY(MathUtil.applyDeadband(-joystick.getLeftX(), 0.1) * MaxSpeed * m_flywheel.speedModifier) // Drive left with negative X (left)
                                .withRotationalRate(MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * MaxAngularRate))); // Drive counterclockwise with negative X (left)
        // @formatter:on
                // Idle while the robot is disabled. This ensures the configured
                // neutral mode is applied to the drive motors while disabled.
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // hopper.setDefaultCommand(new RunCommand(() -> hopper.startAllMotors(1.6,
                // 1.7), hopper));

                // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
                // joystick.b().whileTrue(drivetrain.applyRequest(
                // () -> point.withModuleDirection(
                // new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

                // LED test controls
                // joystick.x().onTrue(Commands.runOnce(() -> m_candle.setLedColor(2, 92, 40)));
                // // gearcat teal!
                // joystick.y().onTrue(Commands.runOnce(() -> m_candle.setRainbowAnimation()));
                // joystick.y().onTrue(new IntakeFuel(m_intake, 1));
                // joystick.pov(0).whileTrue(new FeedFuel(m_feeder));
                // joystick.pov(90).whileTrue(new ShootFuel(m_shooter, 1));
                // joystick.pov(90).whileTrue(new IntakeFuel(m_intake, 1));
                // joystick.rightBumper().whileTrue(new RunCommand(() ->
                // m_candle.colorWithBrightness(
                // Math.sqrt(Math.pow(joystick.getLeftX(), 2) + Math.pow(joystick.getLeftY(),
                // 2))
                // )));

                // joystick.leftTrigger().whileTrue(new RunCommand(() ->
                // m_candle.colorWithBrightness(
                // joystick.getLeftTriggerAxis()
                // )));

                // new Trigger(() -> joystick2.getLeftTriggerAxis() > 0.01)
                // .whileTrue(new RunCommand(() -> m_candle.colorWithBrightness(
                // () -> joystick2.getLeftTriggerAxis())));

                // Change input str to
                // joystick.rightBumper().onTrue(Commands.runOnce(() -> m_candle.cycleFlag()));

                // Run SysId routines when holding back/start and X/Y.
                // Note that each routine should be run exactly once in a single log.
                // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
                // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
                // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
                // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

                // joystick.a().whileTrue(drivetrain.applyRequest(() ->
                // drive.withRotationalRate(getAlignRate() * 0.1)));

                // reset the field-centric heading on left bumper press
                // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
                // drivetrain.seedFieldCentric()));

                drivetrain.registerTelemetry(logger::telemeterize);

                // Reset the field-centric heading on left bumper press.
                joystick.start().onTrue(new InstantCommand(() -> setRobotOrientation()));

                new Trigger(() -> Math.abs(joystick2.getRightX()) > 0.1)
                                .whileTrue(new MoveTurret(m_turret, () -> joystick2.getRightX() * 0.2));

                joystick.povRight().whileTrue(new MoveTurret(m_turret, () -> 0.2));
                joystick.povLeft().whileTrue(new MoveTurret(m_turret, () -> -0.2));

                // joystick.rightBumper().onTrue(new RunHopper(hopper));
                joystick.leftBumper().whileTrue(new RunIntake(m_intake, -3));

                new Trigger(() -> Math.abs(joystick.getLeftTriggerAxis()) > 0.1)
                                .whileTrue(new ShootingSequence(hopper, m_flywheel, m_turret));

                joystick.y().onTrue(new InstantCommand(() -> m_turret.zeroMotorPosition()));
                joystick.back().onTrue(new InstantCommand(() -> m_turret.toggleOverride()))
                                .onFalse(new InstantCommand(() -> m_turret.toggleOverride()));

                closeLogSendable.onChange(closeLog -> {
                        if (closeLog) {
                                this.closeLogFile();
                        }
                });
                // joystick.a().onTrue(new InstantCommand(() -> {
                // ShooterValuesSenable data = (ShooterValuesSenable)
                // SmartDashboard.getData("Shooter Values");
                // if (data != null) {
                // LogValues(data.getDist(), data.getShooterSpeed());
                // }

                // }));
                joystick.a().whileTrue(new AlignTurretToHub(m_turret));
        }

        /**
         * Returns the currently selected autonomous command.
         *
         * @return selected command from the auto chooser
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        /**
         * Sets the initial robot and camera orientation for the primary Limelight.
         */
        public void setRobotOrientation() {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                        if (alliance.get().equals(Alliance.Blue)) {
                                drivetrain.resetPose(new Pose2d());
                                drivetrain.setOperatorPerspectiveForward(new Rotation2d());

                                // GCD
                                LimelightHelpers.SetRobotOrientation("limelight-gcd",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcd", -0.3, 0.25, 0.15, 0, 150,
                                                45);

                                // GCC
                                LimelightHelpers.SetRobotOrientation("limelight-gcc",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble() + 180, 0, 0, 0, 0,
                                                0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.3, -0.25, 0.15, 0, 150,
                                                -45);
                        } else {
                                drivetrain.resetPose(new Pose2d());
                                drivetrain.setOperatorPerspectiveForward(new Rotation2d());

                                // GCD
                                LimelightHelpers.SetRobotOrientation("limelight-gcd",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble() + 180, 0, 0, 0, 0,
                                                0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcd", -0.3, 0.25, 0.15, 0, 150,
                                                45);

                                // GCC
                                LimelightHelpers.SetRobotOrientation("limelight-gcc",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble() + 180, 0, 0, 0, 0,
                                                0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.3, -0.25, 0.15, 0, 150,
                                                -45);
                        }
                }
        }

        /**
         * Logs shooter speed and distance values to the JSON log file.
         *
         * @param shooterSpeed shooter wheel speed
         * @param dist         measured distance to target
         */
        public void LogValues(double shooterSpeed, double dist) {
                JSONObject entry = new JSONObject();
                entry.put("shooterSpeed", shooterSpeed);
                entry.put("distance", dist);
                try {
                        if (writer != null) {
                                writer.write(entry.toJSONString());
                        }
                } catch (IOException e) {
                        System.err.println("Could not write to JSON log file.");
                        e.printStackTrace();
                }
        }

        /**
         * Closes the JSON log file and releases the writer.
         */
        public void closeLogFile() {
                try {
                        if (writer != null) {
                                writer.close();
                                writer = null;
                        }
                } catch (IOException e) {
                        System.err.println("Could not close JSON log file.");
                        e.printStackTrace();
                }
        }
}
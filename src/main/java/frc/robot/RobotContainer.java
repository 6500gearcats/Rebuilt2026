// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AlignTurretToHub;
import frc.robot.commands.BurstFire;
import frc.robot.commands.ClimbPole;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.commands.CoolSnurbo;
import frc.robot.commands.MoveTurret;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootFuel;
import frc.robot.commands.ShootingSequence;
import frc.robot.generated.TunerConstants2;
import frc.robot.subsystems.Climber;
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
import frc.robot.utility.SysIDUtil;

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

        private final CommandXboxController joystick;
        private final CommandPS4Controller pranav = new CommandPS4Controller(0);

        private final CommandXboxController joystick2 = new CommandXboxController(1);
        private final XboxController m_gunner;

        public final CommandSwerveDrivetrain drivetrain = TunerConstants2.createDrivetrain();

        private LedCANdle m_candle = new LedCANdle();

        private final Hopper hopper = new Hopper();

        private final SendableChooser<Command> autoChooser;

        private final Flywheel m_flywheel = new Flywheel();

        private final Turret m_turret = new Turret();

        private final Intake m_intake = new Intake();

        private final Climber m_climber = new Climber();

        private final RangeFinder rangeFinder = new RangeFinder();
        private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
        private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(25).get();

        SysIDUtil m_turretSysID = new SysIDUtil();
        SysIDUtil m_flywheelSysID = new SysIDUtil();

        // Vision
        PhotonVisionIO photonVisionIO;
        private final Vision m_vision;

        SlewRateLimiter filterXLimiter = new SlewRateLimiter(20);
        SlewRateLimiter filterYLimiter = new SlewRateLimiter(20);
        SlewRateLimiter filterRotLimiter = new SlewRateLimiter(20);

        /**
         * Creates the container, initializes logging, chooser options, and vision.
         */
        public RobotContainer() {
                joystick = robotStateMachine.getDriver();
                m_gunner = robotStateMachine.getGunner();
                NamedCommands.registerCommand("IntakeFuel", new RunIntake(m_intake, -1));
                NamedCommands.registerCommand("IntakeFuelJason", new RunIntake(m_intake, -1).withTimeout(5));
                NamedCommands.registerCommand("Intake", new RunIntake(m_intake, -0.1).withTimeout(0.2));
                NamedCommands.registerCommand("IntakeLong", new RunIntake(m_intake, -0.1).withTimeout(0.5));
                NamedCommands.registerCommand("ShootFuel", new ShootingSequence(hopper, m_flywheel, m_turret));
                NamedCommands.registerCommand("ShootFuel3s",
                                new ShootingSequence(hopper, m_flywheel, m_turret).withTimeout(3.2));
                NamedCommands.registerCommand("ShootFuel10s",
                                new ShootingSequence(hopper, m_flywheel, m_turret).withTimeout(10.0));
                NamedCommands.registerCommand("ShootFuel7s",
                                new ShootingSequence(hopper, m_flywheel, m_turret).withTimeout(7.0));
                NamedCommands.registerCommand("ShootFuel5s",
                                new ShootingSequence(hopper, m_flywheel, m_turret).withTimeout(5.0));
                NamedCommands.registerCommand("AlignTurret", new AlignTurretToHub(m_turret));
                NamedCommands.registerCommand("AlignTurret1s", new AlignTurretToHub(m_turret).withTimeout(1));
                NamedCommands.registerCommand("Climb", new ClimbPole(m_climber, 0.1)); // TODO: set auto speed
                NamedCommands.registerCommand("BopBop",
                                new RunCommand(() -> m_intake.deployIntake(-0.3)).withTimeout(0.3)
                                                .andThen(new RunIntake(m_intake, -1).withTimeout(0.3)));
                NamedCommands.registerCommand("SpeedUp", new InstantCommand(() -> m_flywheel.setSpeed(0.7)));
                NamedCommands.registerCommand("ClimbUp2s", new ClimbPole(m_climber, 0.5).withTimeout(2));
                NamedCommands.registerCommand("ClimbDown2s", new ClimbPole(m_climber, -0.5).withTimeout(2));

                SmartDashboard.putNumber("Shoot Speed", 0);

                autoChooser = AutoBuilder.buildAutoChooser("testAuto");
                SmartDashboard.putData("Auto Chooser", autoChooser);
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
                                m_turret.goToZero();
                                // m_turretSysID = new SysIDUtil(m_turret);
                                m_flywheelSysID = new SysIDUtil(m_flywheel);
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
                                m_flywheelSysID = new SysIDUtil(m_flywheel);

                                break;
                        default:
                                m_vision = new Vision();
                                break;
                }
                configureBindings();
                robotStateMachine.bindVision(m_vision);
                robotStateMachine.bindDrivetrain(drivetrain);
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

                // Reset the field-centric heading on left bumper press.
                joystick.start().onTrue(new InstantCommand(() -> setRobotOrientation()));

                // new Trigger(() -> Math.abs(joystick2.getRightX()) > 0.1)
                // .whileTrue(new MoveTurret(m_turret, () -> joystick2.getRightX() * 0.2));

                // joystick2.povRight().whileTrue(new MoveTurret(m_turret, () -> 0.2));
                // joystick2.povLeft().whileTrue(new MoveTurret(m_turret, () -> -0.2));
                new Trigger(() -> Math.abs(m_gunner.getRightTriggerAxis()) > 0.1)
                                .onTrue(new RunCommand(() -> m_intake.deployIntake(-0.3)).withTimeout(0.15)
                                                .andThen(new RunIntake(m_intake, -1).withTimeout(0.1)));
                new POVButton(m_gunner, 90).whileTrue(new MoveTurret(m_turret, () -> 0.2));
                new POVButton(m_gunner, 270).whileTrue(new MoveTurret(m_turret, () -> -0.2));

                // joystick.rightBumper().onTrue(new RunHopper(hopper));
                joystick.rightBumper().whileTrue(new CoolSnurbo(m_flywheel));
                joystick.leftBumper().whileTrue(new RunIntake(m_intake, -3));

                new Trigger(() -> Math.abs(m_gunner.getLeftTriggerAxis()) > 0.1)
                                .whileTrue(new ParallelCommandGroup(new RunCommand(
                                                () -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                                                new BurstFire(hopper, m_flywheel, m_turret, robotStateMachine)))
                                .onFalse(new InstantCommand(
                                                () -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0))
                                                .andThen(new CoolSnurbo(m_flywheel).withTimeout(0.2)));

                joystick.y().onTrue(new InstantCommand(() -> m_turret.zeroMotorPosition()));
                joystick.back().onTrue(new InstantCommand(() -> m_turret.toggleOverride()))
                                .onFalse(new InstantCommand(() -> m_turret.toggleOverride()));

                joystick.a().whileTrue(new AlignTurretToHub(m_turret));
                new JoystickButton(m_gunner, XboxController.Button.kY.value).whileTrue(new ClimbPole(m_climber, 0.5));
                new JoystickButton(m_gunner, XboxController.Button.kA.value).whileTrue(new ClimbPole(m_climber, -0.5));
                new JoystickButton(m_gunner, XboxController.Button.kX.value)
                                .onTrue(new InstantCommand(() -> m_turret.goToZero()));
                new JoystickButton(m_gunner, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new BurstFire(hopper, m_flywheel, robotStateMachine));
                new POVButton(m_gunner, 0).onTrue(new InstantCommand(() -> m_flywheel.incrementMultiplierUp()));

                new POVButton(m_gunner, 180).onTrue(new InstantCommand(() -> m_flywheel.incrementMultiplierDown()));

                if (m_turretSysID.isPresent()) {
                        // Driver Back + A
                        joystick.back().and(joystick.a())
                                        .onTrue(m_flywheelSysID.sysIdAll().get()
                                                        .andThen(new InstantCommand(() -> System.out
                                                                        .println("Get Hoot Logs from TunerX"))));
                }
                if (m_flywheelSysID.isPresent()) {
                        // Driver Back + B
                        joystick.b()
                                        .onTrue(m_flywheelSysID.sysIdAll().get().andThen(new InstantCommand(
                                                        () -> System.out.println("Get Hoot Logs from TunerX"))));
                }
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
                                // drivetrain.resetPose(new Pose2d());
                                // drivetrain.setOperatorPerspectiveForward(new Rotation2d());
                                drivetrain.seedFieldCentric();

                                // GCD
                                LimelightHelpers.SetRobotOrientation("limelight-gcd",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcd", -0.3, 0.25, 0.15, 0, 150,
                                                45);

                                // GCC
                                LimelightHelpers.SetRobotOrientation("limelight-gcc",
                                                drivetrain.getPigeon().getYaw().getValueAsDouble() + 180, 0, 0, 0, 0,
                                                0);

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.3, -0.25, 0.15,
                                                0, 150,
                                                -45);
                        } else {
                                // drivetrain.resetPose(new Pose2d());
                                drivetrain.seedFieldCentric();
                                // drivetrain.setOperatorPerspectiveForward(new Rotation2d());

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

                                LimelightHelpers.setCameraPose_RobotSpace("limelight-gcc", -0.3, -0.25, 0.15,
                                                0, 150,
                                                -45);
                        }
                }
        }

        public void disableInitCode() {
                m_vision.throttleLimelight();
        }

        public void disableExitCode() {
                m_vision.resetLimelightThrottle();
        }
}
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
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AimPrep;
import frc.robot.commands.RunHopper;
import frc.robot.commands.RunIntake;
import frc.robot.commands.ShootWhenReady;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LedCANdle;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.turret.TurretIODisabled;
import frc.robot.subsystems.turret.TurretIOHardware;
import frc.robot.subsystems.turret.TurretIOSim;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.photonvision.PhotonVisionIO;
import frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO;
import frc.robot.superstructure.StateManager;
import frc.robot.utility.SysIDUtil;

/**
 * Central robot wiring for subsystems, commands, and operator bindings.
 */
public class RobotContainer {
        @SuppressWarnings("unused")
        private double speedModify = 1;
        private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
        private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

        private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
                        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
        private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
        private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

        private final CommandXboxController joystick;
        private final CommandXboxController joystick2 = new CommandXboxController(1);
        private final XboxController m_gunner;

        public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

        private LedCANdle m_candle = new LedCANdle();

        private final Hopper hopper = new Hopper();

        private final SendableChooser<Command> autoChooser;

        private final Shooter m_shooter;

        private Turret m_turret;

        private final Intake m_intake = new Intake();

        private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();

        SysIDUtil m_turretSysID = new SysIDUtil();
        SysIDUtil m_flywheelSysID = new SysIDUtil();

        // Vision
        PhotonVisionIO photonVisionIO;
        private final Vision m_vision;

        private final StateManager m_stateManager;

        SlewRateLimiter filterXLimiter = new SlewRateLimiter(20);
        SlewRateLimiter filterYLimiter = new SlewRateLimiter(20);
        SlewRateLimiter filterRotLimiter = new SlewRateLimiter(20);

        /**
         * Creates the container, initializes logging, chooser options, and vision.
         */
        public RobotContainer() {
                m_shooter = robotStateMachine.getShooter();
                joystick = robotStateMachine.getDriver();
                m_gunner = robotStateMachine.getGunner();
                m_stateManager = new StateManager(robotStateMachine);

                // PathPlanner auto commands — shooting/turret stubs re-implemented in Stage 6
                NamedCommands.registerCommand("IntakeFuel", new RunIntake(m_intake, -1));
                NamedCommands.registerCommand("IntakeFuelJason", new RunIntake(m_intake, -1).withTimeout(5));
                NamedCommands.registerCommand("Intake", new RunIntake(m_intake, -0.1).withTimeout(0.2));
                NamedCommands.registerCommand("IntakeLong",
                                new ParallelCommandGroup(new RunIntake(m_intake, -0.1).withTimeout(0.8)));
                NamedCommands.registerCommand("ShootFuel", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("ShootFuel3s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("ShootFuel10s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("ShootFuel7s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("ShootFuel5s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("NewShootFuel3s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("ManualShootFuel3s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("TrenchStartAngle", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("NewShootFuel5s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("NewShootFuel10s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("NewShootFuel4s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("NewShootFuel8s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("AlignTurret", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("AlignTurret1s", Commands.none()); // TODO Stage 6
                NamedCommands.registerCommand("BopBop",
                                new RunCommand(() -> m_intake.deployIntake(-0.3)).withTimeout(0.35)
                                                .andThen(new RunIntake(m_intake, -1).withTimeout(0.3)));
                NamedCommands.registerCommand("SpeedUp", Commands.none()); // TODO Stage 6
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
                                m_vision = new Vision(
                                                drivetrain.rotationSupplier(),
                                                drivetrain.modulePositionsSupplier(),
                                                drivetrain.poseSupplier(),
                                                m_photonVisionIO,
                                                m_photonVisionIO2);
                                m_turret = new Turret(new TurretIOHardware());
                                break;
                        case SIM:
                                SimCameraProperties cameraProp = new SimCameraProperties();
                                cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
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
                                m_turret = new Turret(new TurretIOSim());
                                break;
                        default:
                                m_vision = new Vision();
                                m_turret = new Turret(new TurretIODisabled());
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
        // @formatter:off
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                                .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                                .withRotationalRate(MathUtil.applyDeadband(joystick.getRightX(), 0.1) * MaxAngularRate)));
        // @formatter:on
                final var idle = new SwerveRequest.Idle();
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                joystick.start().onTrue(new InstantCommand(() -> setRobotOrientation()));

                // Intake
                new Trigger(() -> Math.abs(m_gunner.getRightTriggerAxis()) > 0.1)
                                .onTrue(new RunCommand(() -> m_intake.deployIntake(-0.3)).withTimeout(0.35)
                                                .andThen(new RunIntake(m_intake, -1).withTimeout(0.2)));
                joystick.leftBumper().whileTrue(new RunIntake(m_intake, -3));

                // Shooter: rightBumper reverses, gunner left trigger → aim + shoot
                joystick.rightBumper().whileTrue(m_shooter.reverse());

                new Trigger(() -> Math.abs(m_gunner.getLeftTriggerAxis()) > 0.1)
                                .whileTrue(new ParallelCommandGroup(
                                        new RunCommand(() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                                        new AimPrep().build(m_turret, m_shooter, m_stateManager)))
                                .onFalse(new InstantCommand(
                                        () -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

                // Gunner left bumper → feed when ready
                new JoystickButton(m_gunner, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new ShootWhenReady().build(hopper, robotStateMachine));

                // Turret homing
                joystick.y().onTrue(m_turret.home());
                new JoystickButton(m_gunner, XboxController.Button.kX.value)
                                .onTrue(m_turret.home());

                // TODO Stage 6: POV 90/270 for manual turret jog (MoveTurret removed in Stage 5)
                // TODO Stage 6: POV 0/180 for aim tuning
        }

        /**
         * Returns the currently selected autonomous command.
         */
        public Command getAutonomousCommand() {
                return autoChooser.getSelected();
        }

        public void setRobotOrientation() {
                drivetrain.seedFieldCentric();
        }

        public void disableInitCode() {
        }

        public void disableExitCode() {
        }
}

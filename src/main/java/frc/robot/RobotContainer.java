// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.photonvision.simulation.SimCameraProperties;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.FeedFuel;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.ShootFuel;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedCANdle;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.photonvision.PhotonVisionIO;
import frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final XboxController joystick2 = new XboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private LedCANdle m_candle = new LedCANdle();

    private final SendableChooser<Command> autoChooser;

    private final Feeder m_feeder = new Feeder();
    private final Intake m_intake = new Intake();
    private final Shooter m_shooter = new Shooter();

    // Vision
    PhotonVisionIO photonVisionIO = new PhotonVisionIO("photonvision", true, new Translation3d(0.1, 0, 0.5),  new Rotation3d(0, Math.toRadians(-15), 0));
    private final Vision m_vision;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        configureBindings();
        FollowPathCommand.warmupCommand().schedule();
        switch (RobotConstants.currentMode) {
            case REAL:
                PhotonVisionIO m_photonVisionIO = new PhotonVisionIO("photonvision", true, new Translation3d(0.1, 0, 0.5), new Rotation3d(0, Math.toRadians(-15), 0));
                m_vision = new Vision(
                    drivetrain.rotationSupplier(),
                    drivetrain.modulePositionsSupplier(),
                    drivetrain.poseSupplier(),
                    m_photonVisionIO);
                break;
            case SIM:
                // TODO: Add Real Camera Constants to use here
                SimCameraProperties cameraProp = new SimCameraProperties();
                cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
                // Approximate detection noise with average and standard deviation error in pixels.
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
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // LED test controls
        joystick.x().onTrue(Commands.runOnce(() -> m_candle.setLedColor(2, 92, 40))); // gearcat teal!
        joystick.y().onTrue(Commands.runOnce(() -> m_candle.setRainbowAnimation()));
        joystick.y().onTrue(new IntakeFuel(m_intake, 1));
        joystick.pov(0).whileTrue(new FeedFuel(m_feeder));
        joystick.pov(90).whileTrue(new ShootFuel(m_shooter, 1));
        joystick.pov(90).whileTrue(new IntakeFuel(m_intake, 1));
        // joystick.rightBumper().whileTrue(new RunCommand(() -> m_candle.colorWithBrightness(
        //     Math.sqrt(Math.pow(joystick.getLeftX(), 2) + Math.pow(joystick.getLeftY(), 2))
        // )));

        // joystick.leftTrigger().whileTrue(new RunCommand(() -> m_candle.colorWithBrightness(
        //     joystick.getLeftTriggerAxis()
        // )));

        new Trigger(() -> joystick2.getLeftTriggerAxis() > 0.01).whileTrue(new RunCommand(() -> m_candle.colorWithBrightness(
            () -> joystick2.getLeftTriggerAxis())));

        // Change input str to 
        joystick.rightBumper().onTrue(Commands.runOnce(() -> m_candle.cycleFlag()));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //drivetrain.registerTelemetry(logger::telemeterize);

        // Reset the field-centric heading on left bumper press.
        joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }


    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

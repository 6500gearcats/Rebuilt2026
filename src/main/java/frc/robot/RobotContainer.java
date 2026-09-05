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
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.AimPrep;
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
 * Central robot configuration class — the "wiring diagram" that connects subsystems, commands,
 * and operator inputs at startup.
 *
 * <h2>What Goes Here</h2>
 * In WPILib's command-based framework, {@code RobotContainer} is where:
 * <ul>
 *   <li>Every subsystem is instantiated once and only once.
 *   <li>PathPlanner {@link NamedCommands} are registered so auto paths can trigger them by name.
 *   <li>Controller buttons are bound to commands via {@link #configureBindings()}.
 *   <li>The autonomous command chooser is built and posted to SmartDashboard.
 * </ul>
 *
 * <h2>IO Mode Selection</h2>
 * Hardware-specific implementations are chosen in the constructor via a switch on
 * {@link Constants.RobotConstants#currentMode}:
 * <ul>
 *   <li>{@code REAL} — TurretIOHardware + PhotonVisionIO: real CAN motors and cameras.
 *   <li>{@code SIM} — TurretIOSim + PhotonVisionSimIO: simulated motors and a virtual camera.
 *   <li>{@code default} — TurretIODisabled + empty Vision: used in unit tests or replay;
 *       all hardware calls are no-ops.
 * </ul>
 *
 * <h2>Controller Layout</h2>
 * Two Xbox controllers are in use:
 * <ul>
 *   <li><b>Driver (joystick, port 0)</b> — drives the swerve base, resets field orientation,
 *       reverses the shooter for unjamming, and runs intake from the left bumper.
 *   <li><b>Gunner (joystick2 / m_gunner, port 1)</b> — operates the entire scoring mechanism:
 *       right trigger for intake, left trigger for aim prep, left bumper for fire-when-ready,
 *       X button to home the turret, and POV for manual turret jog.
 * </ul>
 *
 * <h2>PathPlanner NamedCommand Integration</h2>
 * PathPlanner auto paths reference named commands by string key. Every {@code NamedCommands.register}
 * call in the constructor defines one of those strings. The shooting variants ({@code ShootFuelNs})
 * all use {@link #aimAndShoot()} wrapped in a timeout so the auto path continues after N seconds
 * regardless of whether a shot was fired — a safety timeout for missed shots in competition.
 */
public class RobotContainer {
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
         * Constructs the container: instantiates all subsystems, registers PathPlanner named commands,
         * selects hardware vs. simulation IO, wires bindings, and links vision + drivetrain to the
         * robot state machine.
         *
         * <p>Execution order matters here:
         * <ol>
         *   <li>Shooter and controllers are pulled from the singleton {@link RobotStateMachine} so that
         *       one object owns those references — the state machine needs them for game logic.
         *   <li>{@link NamedCommands} must be registered <em>before</em> {@code AutoBuilder.buildAutoChooser}
         *       is called, or PathPlanner will not know about the commands referenced in auto paths.
         *   <li>The IO mode switch must run before {@link #configureBindings()} because bindings reference
         *       {@code m_turret}, which is assigned in the switch.
         *   <li>After bindings, vision and drivetrain are bound to the state machine so it can start
         *       fusing pose estimates.
         * </ol>
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
                NamedCommands.registerCommand("ShootFuel",         aimAndShoot());
                NamedCommands.registerCommand("ShootFuel3s",       aimAndShoot().withTimeout(3.0));
                NamedCommands.registerCommand("ShootFuel5s",       aimAndShoot().withTimeout(5.0));
                NamedCommands.registerCommand("ShootFuel7s",       aimAndShoot().withTimeout(7.0));
                NamedCommands.registerCommand("ShootFuel10s",      aimAndShoot().withTimeout(10.0));
                NamedCommands.registerCommand("NewShootFuel3s",    aimAndShoot().withTimeout(3.0));
                NamedCommands.registerCommand("NewShootFuel4s",    aimAndShoot().withTimeout(4.0));
                NamedCommands.registerCommand("NewShootFuel5s",    aimAndShoot().withTimeout(5.0));
                NamedCommands.registerCommand("NewShootFuel8s",    aimAndShoot().withTimeout(8.0));
                NamedCommands.registerCommand("NewShootFuel10s",   aimAndShoot().withTimeout(10.0));
                NamedCommands.registerCommand("ManualShootFuel3s",
                        new ShootWhenReady().build(hopper, robotStateMachine).withTimeout(3.0));
                NamedCommands.registerCommand("TrenchStartAngle",  m_turret.home());
                NamedCommands.registerCommand("AlignTurret",       m_turret.track(m_stateManager));
                NamedCommands.registerCommand("AlignTurret1s",     m_turret.track(m_stateManager).withTimeout(1.0));
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
         * Wires all operator inputs to commands.
         *
         * <h2>Driver Controls (port 0)</h2>
         * <ul>
         *   <li><b>Default command</b>: Field-centric swerve drive. Left stick = translation (X/Y),
         *       right stick = rotation. A 10% deadband removes stick drift.
         *   <li><b>Start button</b>: Resets the field-relative forward direction to match the robot's
         *       current physical heading. Used when the gyro drifts or after a tip event.
         *   <li><b>Left bumper</b>: Runs the intake rollers for ball collection (duty cycle −3 = full
         *       reverse = into robot). Runs {@code whileTrue} so rollers stop when bumper is released.
         *   <li><b>Right bumper</b>: Reverses the shooter wheels to clear ball jams.
         * </ul>
         *
         * <h2>Gunner Controls (port 1)</h2>
         * <ul>
         *   <li><b>Right trigger (&gt; 10%)</b>: Intake sequence — briefly deploys the intake arm
         *       (−0.3 for 350 ms) then runs intake rollers (−1 for 200 ms). The sequence uses
         *       {@code andThen} so both steps complete in order. {@code onTrue} runs the entire
         *       sequence once per trigger press.
         *   <li><b>Left trigger (&gt; 10%)</b>: Aim prep — runs {@link AimPrep} in parallel with a
         *       driver rumble (so the driver knows the gunner is aiming). {@code whileTrue} keeps both
         *       running until the trigger is released. The {@code onFalse} binding cuts rumble when the
         *       trigger releases.
         *   <li><b>Left bumper</b>: Fire when ready — runs {@link ShootWhenReady}, which blocks until
         *       the shooter is on-target and then runs the hopper. Designed to run simultaneously with
         *       the left trigger (aim prep).
         *   <li><b>X button</b>: Homes the turret to the forward-facing angle. Useful if the turret
         *       gets lost or before starting an auto path that assumes a known turret position.
         *   <li><b>POV right (90°)</b>: Jogs the turret clockwise at 0.5 rot/s while held.
         *   <li><b>POV left (270°)</b>: Jogs the turret counter-clockwise at 0.5 rot/s while held.
         *       Jog uses {@link frc.robot.subsystems.turret.Turret#jog Turret.jog}, which increments
         *       the MotionMagic position reference each loop cycle — smooth mechanical response.
         * </ul>
         *
         * <h2>Disable-Mode Drive Safety</h2>
         * When the robot is disabled, the drivetrain runs an {@code Idle} request (wheels unlocked,
         * no active output) so technicians can push the robot without fighting the hold brake.
         * {@code ignoringDisable(true)} lets this run even while the DS reports disabled.
         */
        private void configureBindings() {
        // @formatter:off
        // Default: field-centric drive. Left Y = forward, left X = strafe, right X = rotate.
        // applyDeadband removes stick noise below 10% of full deflection.
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(MathUtil.applyDeadband(joystick.getLeftY(), 0.1) * MaxSpeed)
                                .withVelocityY(MathUtil.applyDeadband(joystick.getLeftX(), 0.1) * MaxSpeed)
                                .withRotationalRate(MathUtil.applyDeadband(joystick.getRightX(), 0.1) * MaxAngularRate)));
        // @formatter:on
                final var idle = new SwerveRequest.Idle();
                // When disabled: unlock modules so the robot can be pushed manually on the field.
                RobotModeTriggers.disabled().whileTrue(
                                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

                // Driver Start: re-zero gyro field-relative heading to current robot heading.
                joystick.start().onTrue(new InstantCommand(() -> setRobotOrientation()));

                // Gunner right trigger: intake sequence — deploy arm briefly, then run rollers.
                new Trigger(() -> Math.abs(m_gunner.getRightTriggerAxis()) > 0.1)
                                .onTrue(new RunCommand(() -> m_intake.deployIntake(-0.3)).withTimeout(0.35)
                                                .andThen(new RunIntake(m_intake, -1).withTimeout(0.2)));
                // Driver left bumper: intake rollers only (no deploy — intake already out, or for re-intake).
                joystick.leftBumper().whileTrue(new RunIntake(m_intake, -3));

                // Driver right bumper: reverse flywheel to clear jams.
                joystick.rightBumper().whileTrue(m_shooter.reverse());

                // Gunner left trigger: aim prep (turret tracking + shooter spin-up) + driver rumble feedback.
                // whileTrue: both keep running until trigger is released.
                new Trigger(() -> Math.abs(m_gunner.getLeftTriggerAxis()) > 0.1)
                                .whileTrue(new ParallelCommandGroup(
                                        new RunCommand(() -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 1)),
                                        new AimPrep().build(m_turret, m_shooter, m_stateManager)))
                                .onFalse(new InstantCommand(
                                        () -> joystick.setRumble(GenericHID.RumbleType.kBothRumble, 0)));

                // Gunner left bumper: feed ball when shooter + turret are on-target.
                // Runs in parallel with left-trigger aim prep — no scheduling conflict because
                // ShootWhenReady's waitUntil phase has no subsystem requirement.
                new JoystickButton(m_gunner, XboxController.Button.kLeftBumper.value)
                                .whileTrue(new ShootWhenReady().build(hopper, robotStateMachine));

                // Turret home: both driver Y and gunner X home the turret to the forward angle.
                // Driver can home during auto recovery; gunner homes before a trench run.
                joystick.y().onTrue(m_turret.home());
                new JoystickButton(m_gunner, XboxController.Button.kX.value)
                                .onTrue(m_turret.home());

                // Gunner POV: manual turret jog at 0.5 rot/s. Each press holds until released.
                joystick2.pov(90).whileTrue(m_turret.jog(0.5));
                joystick2.pov(270).whileTrue(m_turret.jog(-0.5));
        }

        /**
         * Builds the standard auto shooting command: aim prep and hopper feeding run in parallel,
         * so the turret and shooter converge at the same time the hopper waits for the ready signal.
         *
         * <p>All timed shooting named commands ({@code ShootFuelNs}) wrap this in a
         * {@code withTimeout(N)} so the auto path continues after N seconds even if the shot was
         * missed. {@code ManualShootFuel3s} omits {@link AimPrep} — it feeds whenever the human
         * driver has already set up the shot externally.
         */
        private Command aimAndShoot() {
                return Commands.parallel(
                        new AimPrep().build(m_turret, m_shooter, m_stateManager),
                        new ShootWhenReady().build(hopper, robotStateMachine));
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

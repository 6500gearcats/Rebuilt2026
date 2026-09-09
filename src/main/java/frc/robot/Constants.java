// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  /**
   * Which physical AprilTag field variant this robot's pose estimation and hub-targeting are
   * built against. Exposed as its own constant (rather than inlined into
   * {@link #APRIL_TAG_FIELD_LAYOUT}'s initializer) so {@link frc.robot.Robot} can log the
   * active layout by name at startup — see the log line in {@code Robot()}.
   *
   * <p><b>This is the single field layout used everywhere in this codebase</b> —
   * {@link VisionConstants#kTagLayout} is repointed at this same value. Found and fixed
   * 2026-09-09: the two constants previously loaded two different field variants
   * ({@code k2026RebuiltAndymark} here, {@code kDefaultField} in {@code VisionConstants}),
   * so the robot localized against one map and aimed at a hub derived from the other — a
   * constant offset that vision corrections could never detect or correct, because each half
   * was internally self-consistent. See {@code plans/review_plan.md} R1-A1 for the full
   * writeup.
   */
  public static final AprilTagFields FIELD_LAYOUT_SOURCE = AprilTagFields.k2026RebuiltAndymark;

  public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout
      .loadField(FIELD_LAYOUT_SOURCE);

  /** Selects the robot operating mode (real vs. simulation) at startup. */
  public static class RobotConstants {
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;
  }

  /** Indicates whether the robot is running on real hardware or in simulation. */
  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,
  }

  /**
   * Drive base physical constants and CAN IDs for the MAXSwerve drivetrain.
   *
   * <p>Note: CAN IDs here are for the older REV MAXSwerve configuration. The robot
   * is being migrated to CTRE TalonFX swerve (see {@link frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain});
   * these constants remain for legacy code compatibility.
   */
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.6; // 4.5
    public static final double kNormalSpeedMetersPerSecond = 1.5; // 0.85
    public static final double kMaxAngularSpeed = 1 * Math.PI; // radians per second (was 0.75)

    // turbo
    public static final double kTurboModeModifier = 7.0;
    public static double kTurboAngularSpeed = 2.0;

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // caclulate front wheel offset angles using math. Similar angles means we can
    // just use width/length as opposite/adjacent
    public static final double theta = Math.atan((kTrackWidth) / (kWheelBase));

    public static final double kFrontLeftChassisAngularOffset = -theta * 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI - 2 * theta;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 4;
    public static final int kFrontRightDrivingCanId = 2;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 6;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;

  }

  /** Per-module gear ratios, encoder conversion factors, and PID gains for MAXSwerve modules. */
  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth
    // will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Invert the turning encoder, since the output shaft rotates in the opposite
    // direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction; // meters
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 40; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  /** Operator interface — USB port assignments for driver and gunner controllers. */
  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kGunnerControllerPort = 1;
  }

  /**
   * PathPlanner and autonomous-mode constants.
   *
   * <p>{@link #config} is loaded from the PathPlanner GUI settings file at startup.
   * If loading fails (e.g., the file was not deployed), autos will not run and an error
   * is printed to the Driver Station console.
   */
  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    public static RobotConfig config;
    static {
      try {
        config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
        System.err.println("[Constants] RobotConfig load failed — deploy PathPlanner GUI settings to fix. Autos will not run.");
        e.printStackTrace();
      }
    }

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  /** Free-speed reference for REV NEO motors (used in MAXSwerve drive feed-forward). */
  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  /** IMU configuration — tilt angle for the gyro relative to the robot frame. */
  public static final class GyroConstants {
    public static final double kTiltPitch = 65; // 11? tilt angle=
  }

  /**
   * Vision camera calibration constants.
   *
   * <p>{@link #kSingleTagStdDevs} and {@link #kMultiTagStdDevs} are placeholder values
   * (marked with "Fake values") — measure experimentally with the physical robot in Stage 8.
   */
  public static class VisionConstants {
    public static final String kCameraNameTag = "Microsoft_LifeCam_HD-3000";
    public static final String kCameraNameNote = "Microsoft_LifeCam_VX-5000";
    public static final String kCameraNameGlobal = "Global_Shutter_Camera";
    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5),
        new Rotation3d(0, 0, 180));

    /**
     * The layout of the AprilTags on the field — repointed at {@link Constants#APRIL_TAG_FIELD_LAYOUT}
     * (2026-09-09) so vision pose estimation and hub-targeting agree on one field map. The
     * constant name is kept as-is (rather than removed) because {@code PhotonVisionIO} and
     * {@code PhotonVisionSimIO} both {@code import static} it by this name.
     *
     * <p><b>Previously</b> this loaded {@code AprilTagFields.kDefaultField} — a different,
     * independent field variant from {@code APRIL_TAG_FIELD_LAYOUT}'s
     * {@code k2026RebuiltAndymark}. That mismatch is preserved here only as history, not as a
     * live option: running two field layouts simultaneously was the bug (see
     * {@link Constants#FIELD_LAYOUT_SOURCE}'s Javadoc). If this robot ever needs to play on a
     * field using the plain/default AndyMark layout instead, change
     * {@link Constants#FIELD_LAYOUT_SOURCE}, not this line.
     */
    public static final AprilTagFieldLayout kTagLayout = Constants.APRIL_TAG_FIELD_LAYOUT;

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // ! (Fake values. Experiment and determine estimation noise on an actual
    // robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(.5, .5, Units.degreesToRadians(20));
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(.5, .5, Units.degreesToRadians(5));
    public static final double CAMERA_HEIGHT_METERS = Units.inchesToMeters(27);

    public static final double TARGET_HEIGHT_METERS = Units.feetToMeters(5);
    // Angle between horizontal and the camera.
    public static final double CAMERA_PITCH_RADIANS = Units.degreesToRadians(-28);

    // How far from the target we want to be
    public static final double GOAL_RANGE_METERS = Units.feetToMeters(3);

  }

  /**
   * Legacy CAN IDs for motors not yet migrated to subsystem-specific constants classes.
   * These IDs are from the Hackbots configuration and require re-assignment in Stage 8-1.
   */
  public static class MotorConstants {
    public static final int kTurretYawMotorID = 12;
    public static final int kShooterMotorRightID = 13;
    public static final int kShooterMotorLeftID = 14;

    public static final int kIntakeMotorID = 20;
    public static final int kIntakeDeployMotorID = 21;
    public static final int kIndexerMotorID = 22;
    public static final int kKickerMotorID = 23;
  }

  /**
   * Robot-to-turret geometric transform constants.
   *
   * <p>{@link #ROBOT_TO_TURRET_BASE} is a placeholder — measure from CAD and confirm
   * physically in Stage 8-2.
   */
  public static class TurretConstants {
    public static final double kTurretTransformMetersX = 0.1524;
    public static final double kTurretTransformIMetersY = 0.0635;
    /** Turret base offset from robot center — measure from CAD in Stage 8-2 */
    public static final Transform2d ROBOT_TO_TURRET_BASE = new Transform2d(
        new Translation2d(0.0, 0.0), new Rotation2d()); // PLACEHOLDER
  }


}

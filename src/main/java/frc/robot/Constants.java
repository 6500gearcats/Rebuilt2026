// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
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
 *
 * <p><b>Dead-code sweep, 2026-09-09 (see {@code plans/review_plan.md} R4-C10):</b> six
 * entire nested classes were removed here — {@code DriveConstants}, {@code ModuleConstants},
 * {@code OIConstants}, {@code AutoConstants}, {@code NeoMotorConstants}, {@code GyroConstants}
 * — after `grep`-confirming zero external references to any of them. Two are worth recording
 * even though the classes themselves are gone:
 * <ul>
 *   <li>{@code DriveConstants}/{@code ModuleConstants} configured the robot's original REV
 *       MAXSwerve drivetrain (SPARK MAX CAN IDs, NEO free speed, etc.) — fully superseded by
 *       the CTRE TalonFX swerve in {@code TunerConstants2} and
 *       {@link frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain}.
 *   <li>{@code AutoConstants} held a second, independent implementation of "load
 *       {@code RobotConfig} safely and report failure" — the exact fix recorded as done in
 *       Stage 0 / ISSUES.md M-3. That implementation <b>never executed</b>: nothing in the
 *       codebase ever referenced the {@code AutoConstants} class, and Java only runs a
 *       class's static initializer on first reference, so its try/catch never ran even once.
 *       The actually-reachable equivalent lives in
 *       {@link frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain#configureAutoBuilder()},
 *       which loads {@code RobotConfig.fromGUISettings()} itself and reports failure via
 *       {@code DriverStation.reportError}. M-3's fix is real, just not where the master plan's
 *       tracker said it was — see {@code plans/AUDIT_PROGRESS.md} R-1 for the correction.
 * </ul>
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
   * Vision camera calibration constants.
   */
  public static class VisionConstants {
    public static final String kCameraNameTag = "Microsoft_LifeCam_HD-3000";
    public static final String kCameraNameNote = "Microsoft_LifeCam_VX-5000";
    public static final String kCameraNameGlobal = "Global_Shutter_Camera";

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

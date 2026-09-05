package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.aiming.AimParams;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.superstructure.StateManager;
import frc.robot.util.OnboardLogger;
import frc.robot.vision.localization.LocalizationConstants;

/**
 * Turret subsystem — a single-axis yaw mechanism that rotates the shooter to face the hub.
 *
 * <h2>Hardware Overview</h2>
 * The turret uses one TalonFX motor driving a gear reduction ({@link TurretConstants#kGearRatio})
 * with a single CANcoder absolute encoder for position. A single absolute encoder means no
 * homing routine is needed after power-on — the turret knows its absolute angle immediately.
 * The encoder's magnetic offset ({@link TurretConstants#kEncoderConfig}) is calibrated so that
 * reading 0 corresponds to the turret facing directly forward on the robot.
 *
 * <h2>Travel Limits</h2>
 * The turret can rotate between {@link TurretConstants#kMinAngle} (−0.5 rotations = −180°) and
 * {@link TurretConstants#kMaxAngle} (+0.5 rotations = +180°). TalonFX software limits enforce
 * this in hardware. The {@link #findCC} algorithm (see below) always chooses a target within
 * these limits, so the motor never hits a soft-limit stop during normal tracking.
 *
 * <h2>IO Layer</h2>
 * All hardware interaction goes through {@link TurretIO}. Three implementations exist:
 * <ul>
 *   <li>{@link TurretIOHardware} — real robot (TalonFX + CANcoder).
 *   <li>{@link TurretIOSim} — WPILib simulation (simulated motor physics).
 *   <li>{@link TurretIODisabled} — no-op; used in disabled mode and test environments.
 * </ul>
 * The active implementation is selected in {@link frc.robot.RobotContainer} based on
 * {@link frc.robot.Constants.RobotConstants#currentMode}.
 *
 * <h2>Motion Profile</h2>
 * The motor uses TalonFX MotionMagic in Slot 0: a trapezoidal velocity profile up to
 * {@link TurretConstants#kMaxSpeed} rot/s with {@link TurretConstants#kMaxAcceleration}
 * rot/s². This gives smooth, controlled slewing without overshoot.
 */
public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputs inputs;

  private final Alert calibrationAlert =
      new Alert("Turret not calibrated successfully", AlertType.kError);

  /**
   * True while a {@link #track(StateManager)} command is running. Used by {@link #tracked} to
   * distinguish "at position because tracking" from "at position because we happened to be there".
   */
  private boolean tracking;

  /**
   * The current position reference sent to the motor controller. Updated by {@link #setPosition}
   * every loop while a positioning command is active. Stored here so {@link #jog} can read and
   * increment it without re-reading the motor encoder.
   */
  private Angle reference = TurretConstants.kHomePosition;

  /**
   * Constructs the turret, calibrates the encoder, and registers telemetry + SmartDashboard buttons.
   *
   * <p>{@link TurretIO#calibrate()} reads the CANcoder absolute position and sets the TalonFX
   * internal encoder to match, establishing the absolute reference. Calibration also runs
   * automatically when the robot is disabled (via {@link RobotModeTriggers#disabled()}) to
   * recover from any drift.
   *
   * @param io the hardware/sim/disabled implementation to use
   */
  public Turret(TurretIO io) {
    super();
    this.io = io;
    inputs = new TurretIOInputs();
    io.calibrate();

    // Expose one-click commands on SmartDashboard for drive team use.
    SmartDashboard.putData("Turret/Home", home());
    Command calibrate = runOnce(io::calibrate).ignoringDisable(true);
    SmartDashboard.putData("Turret/Calibrate", calibrate);

    // Re-calibrate every time the robot is disabled in case the encoder drifted during a match.
    RobotModeTriggers.disabled().onTrue(calibrate);

    OnboardLogger log = new OnboardLogger("Turret");
    log.registerBoolean("Ready", ready());
    log.registerBoolean("Tracking", () -> tracking);
    log.registerMeasurement("Reference", () -> reference, Rotations);
  }

  /** Reads sensor values from the IO layer into {@code inputs} each robot loop (20 ms). */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    // Show an error on the driver station if calibration failed at startup or after disable.
    calibrationAlert.set(!inputs.calibrated);
  }

  /**
   * Returns a command that continuously aims the turret at the scoring hub, accounting for robot
   * rotation and ball lead compensation.
   *
   * <h2>Algorithm Each Loop Cycle</h2>
   * <ol>
   *   <li>Get the robot's current field-relative heading from {@link StateManager#robotPose()}.
   *   <li>Get lead-compensated shot parameters from {@link StateManager#aimParams()}, which runs
   *       the full {@code LeadCompensator → ToFAim} pipeline.
   *   <li>If the shot is feasible ({@link AimParams#isOk()}), compute the required turret angle:
   *       <pre>
   *         mechanismAngle = (fieldYaw − robotHeading) + kForwards
   *       </pre>
   *       where {@code fieldYaw} is the field-frame direction to the hub, {@code robotHeading} is
   *       the robot's heading, and {@code kForwards} is the encoder offset for the forward
   *       direction. The subtraction converts field-frame to robot-relative, then {@code kForwards}
   *       maps "robot-forward" to the encoder's zero point.
   *   <li>Set the position reference. The tracking flag passed to {@link #setPosition} is
   *       {@code true} when the shot is not yet ready — this enables a wider position tolerance
   *       in the motion profile for faster slewing, tightening to the final tolerance only when
   *       the shooter is also at speed.
   * </ol>
   *
   * <p>The command uses {@link SubsystemBase#run run()} so it repeats every loop. When it ends
   * (trigger released or interrupted), {@code tracking} is set to {@code false} via
   * {@code finallyDo} so the {@link #tracked} trigger correctly reports not-tracking.
   *
   * @param state provides robot pose and aim parameters from the state machine
   */
  public Command track(StateManager state) {
    return Commands.sequence(
        this.run(() -> {
          tracking = true;
          Rotation2d robot = state.robotPose().getRotation();
          AimParams params = state.aimParams();
          if (!params.isOk()) {
            // No viable shot (out of range, no AprilTag, impossible trajectory) — hold position.
            return;
          }
          // Convert field-frame aim yaw to a robot-relative mechanism angle.
          // params.yaw is in field coordinates; subtracting robot heading gives robot-relative.
          // Adding kForwards accounts for the encoder's zero being at "robot forward".
          Angle mechanismAngle = params.yaw.minus(robot).getMeasure().plus(TurretConstants.kForwards);
          // Use coarse tolerance (tracking=true) until shooter is also ready; tight otherwise.
          setPosition(mechanismAngle, !state.shootReady.getAsBoolean());
        }))
        .finallyDo(() -> tracking = false);
  }

  /**
   * Returns a command that sends the turret to its home position and waits until it arrives.
   *
   * <p>Home is defined by {@link TurretConstants#kHomePosition}. The command completes when
   * {@link #ready()} is true — i.e., when the turret is within {@link TurretConstants#kTolerance}
   * of the reference. Used for auto-path setup ("TrenchStartAngle" named command) and as a
   * driver/gunner quick-reset (joystick Y and gunner X buttons).
   */
  public Command home() {
    return Commands.sequence(
        runOnce(() -> setPosition(TurretConstants.kHomePosition, false)),
        Commands.waitUntil(ready()));
  }

  /**
   * Returns a command that drives the turret to the forward-facing position (hub side of the
   * field when starting auton) and waits until arrived.
   */
  public Command forwards() {
    return Commands.sequence(
        runOnce(() -> setPosition(TurretConstants.kForwards, false)),
        Commands.waitUntil(ready()));
  }

  /**
   * Returns a {@link Trigger} that is {@code true} while the turret is within
   * {@link TurretConstants#kTolerance} of its current position reference.
   *
   * <p>This trigger is used internally by {@link #home()} and {@link #forwards()} to know when
   * to declare the movement complete.
   */
  public Trigger ready() {
    return new Trigger(() -> {
      double delta = inputs.position.minus(inputs.reference).baseUnitMagnitude();
      return Math.abs(delta) <= TurretConstants.kTolerance.baseUnitMagnitude();
    });
  }

  /**
   * Returns a {@link Trigger} that is {@code true} when the turret is actively tracking
   * ({@link #track} is running) AND within the aim tolerance specified in the current
   * {@link AimParams}.
   *
   * <p>The tolerance {@code params.deltaYaw} comes from the aim pipeline and can be tighter or
   * looser depending on how accurate the shot needs to be at the current distance.
   *
   * <p>Used by {@link frc.robot.RobotStateMachine#isShootReady()} (via the {@code Shooter} tracked
   * trigger), which gates the hopper in {@link frc.robot.commands.ShootWhenReady}.
   *
   * @param params supplier of current aim parameters; called each time the trigger is evaluated
   */
  public Trigger tracked(Supplier<AimParams> params) {
    return new Trigger(() -> {
      double delta = inputs.position.minus(inputs.reference).baseUnitMagnitude();
      double epsilon = params.get().deltaYaw.getMeasure().baseUnitMagnitude();
      return Math.abs(delta) <= epsilon && tracking;
    });
  }

  /**
   * Returns the current turret pose in field coordinates, composed from the robot pose and the
   * fixed turret mounting offset ({@link TurretConstants#kOffset}).
   *
   * <p>This 3D pose is used as the "shooter position" in the aiming pipeline so the aim solver
   * computes angles from the actual launch point rather than the robot center.
   *
   * @param robotPose current field-relative robot pose from odometry/vision
   */
  public Pose3d turretPose(Pose2d robotPose) {
    return new Pose3d(robotPose).transformBy(TurretConstants.kOffset);
  }

  /**
   * Returns a command that jogs the turret at a fixed speed by incrementing the position
   * reference each loop cycle.
   *
   * <p>Since the new Hackbots {@link TurretIO} interface only exposes position control (no direct
   * voltage or speed command), manual jog is implemented as a small position step per loop:
   * <pre>
   *   reference += rotationsPerSecond × 0.02 s     (0.02 s = one 50 Hz loop)
   * </pre>
   * This increments the reference smoothly at the specified angular rate, and MotionMagic
   * follows it with the configured cruise velocity and acceleration. When the command ends
   * (button released), the reference stops updating and the turret holds its last position.
   *
   * <p>Bound to gunner POV right (+0.5 rot/s) and POV left (−0.5 rot/s) in
   * {@link frc.robot.RobotContainer}. Soft limits in the TalonFX configuration prevent the turret
   * from jogging past {@link TurretConstants#kMinAngle} or {@link TurretConstants#kMaxAngle}.
   *
   * @param rotationsPerSecond positive = counter-clockwise; negative = clockwise. Typical value
   *                           ±0.5 rot/s = ±180°/s (reaches end of travel in ~1 s)
   */
  public Command jog(double rotationsPerSecond) {
    return this.run(() -> setPosition(reference.plus(Rotations.of(rotationsPerSecond * 0.02)), false));
  }

  /**
   * Selects the motor position reference using the <em>closest congruent</em> algorithm, then
   * sends it to the IO layer.
   *
   * <p>The {@link #findCC} algorithm handles multi-turn wrapping: if the turret is currently at
   * 2.3 rotations and the target angle is 0.7 rotations (both equivalent mod 1), findCC returns
   * the closest equivalent in [min, max] to avoid an unnecessary 270° trip.
   *
   * @param position  target angle in rotations (will be mapped to the closest congruent value)
   * @param tracking  {@code true} → use wider slew tolerance for faster motion during initial
   *                  tracking; {@code false} → hold tight tolerance for final positioning
   */
  private void setPosition(Angle position, boolean tracking) {
    Angle min = TurretConstants.kMinAngle;
    Angle max = TurretConstants.kMaxAngle;
    reference = Rotations.of(findCC(
        inputs.position.in(Rotations),
        position.in(Rotations),
        min.in(Rotations),
        max.in(Rotations)));
    io.setPosition(reference);
  }

  /**
   * Returns the <em>closest congruent</em> value to {@code position} for a given
   * {@code reference} angle, constrained to [{@code min}, {@code max}].
   *
   * <h2>What "Congruent" Means Here</h2>
   * A single absolute encoder reports a value in [0, 1) rotations. But the physical turret can
   * turn multiple full rotations (its range is about ±0.5 rot = ±180°). The motor encoder tracks
   * total rotations (e.g., 2.3), not just the fractional part. Two angles are <em>congruent</em>
   * if they differ by a whole number of rotations — they represent the same physical turret
   * orientation.
   *
   * <h2>Why This Matters</h2>
   * Suppose the turret is at 2.7 rotations and the desired orientation is 0.3 rotations (i.e.,
   * the same as 1.3, 2.3, 3.3, …). The wrong choice is 0.3 — that requires driving from 2.7
   * almost back to 0 (a 2.4 rotation trip). The right choice is 2.3, which is just 0.4 rotations
   * away. This avoids spinning the turret the "long way" and hitting a soft limit.
   *
   * <h2>Algorithm</h2>
   * <ol>
   *   <li>Bring the reference into [min, max] by adding or subtracting 1 rotation at a time.
   *   <li>If still out of range, clamp to max.
   *   <li>If the reference is already within 0.5 rotations of the current position, return it
   *       (it's already the closest congruent value).
   *   <li>Otherwise, try shifting by ±1 rotation toward the current position, stopping when we
   *       find the shift that puts the error below 0.5 rotations.
   * </ol>
   *
   * <p>This is a package-private static so it can be unit-tested independently of hardware.
   *
   * @param position  current multi-turn encoder position (e.g., 2.7)
   * @param reference desired orientation as a fractional rotation in [0, 1) (e.g., 0.3)
   * @param min       minimum allowed encoder position (e.g., −0.5)
   * @param max       maximum allowed encoder position (e.g., +0.5)
   * @return the closest value to {@code position} that is congruent to {@code reference} and
   *         within [{@code min}, {@code max}]
   */
  protected static double findCC(double position, double reference, double min, double max) {
    // Step 1: Bring the reference into [min, max] by adding/subtracting whole rotations.
    while (reference > max) reference -= 1.0;
    while (reference < min) reference += 1.0;

    // If still out of range even after wrapping, clamp to the limit.
    if (reference > max) return max;

    // Step 2: If the error is less than half a rotation, this is already the closest equivalent.
    double error = Math.abs(reference - position);
    if (error < 0.5) return reference;

    // Step 3: Walk toward the current position by adding/subtracting 1 rotation at a time.
    // Stop when we find the first equivalent that is within 0.5 rotations (i.e., closest).
    double offset = (reference > position) ? -1 : 1;
    while (true) {
      double newReference = reference + offset;
      if (newReference > max || newReference < min) break; // Would leave the valid range.
      double newError = Math.abs(newReference - position);
      if (Math.abs(newError) < 0.5) return newReference;
      reference = newReference;
    }
    return reference;
  }

  /**
   * Returns the transform from robot origin to the turret camera in field coordinates, accounting
   * for the current turret angle.
   *
   * <p>Used by the vision subsystem to transform camera observations into field coordinates.
   * The transform is composed of:
   * <ol>
   *   <li>{@link TurretConstants#kOffset} — fixed robot-to-turret-base transform (from CAD).
   *   <li>A rotation by the current turret angle around the yaw axis.
   *   <li>{@link LocalizationConstants#kTurretAoRToTurretCameraOffset} — camera mounting offset
   *       on the turret (measured in Stage 8-2, currently a placeholder).
   * </ol>
   */
  public Transform3d turretCameraOffset() {
    Transform3d turretRelative =
        new Transform3d(Translation3d.kZero,
            new Rotation3d(new Rotation2d(inputs.position.minus(TurretConstants.kForwards))))
            .plus(LocalizationConstants.kTurretAoRToTurretCameraOffset);
    return TurretConstants.kOffset.plus(turretRelative);
  }
}

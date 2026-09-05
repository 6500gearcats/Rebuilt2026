package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.aiming.AimParams;
import frc.robot.aiming.AimParams.SpeedControl;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;
import frc.robot.util.OnboardLogger;

/**
 * Shooter subsystem — two flywheel motors for launch velocity and a servo/motor hood for launch
 * angle.
 *
 * <h2>Responsibilities</h2>
 * <ul>
 *   <li>Accept a stream of {@link AimParams} from the aiming pipeline and drive the wheels and
 *       hood toward the required setpoints each loop.
 *   <li>Expose a {@link #tracked(Supplier)} trigger so the rest of the system knows when both
 *       wheel speed and hood angle are within tolerance.
 *   <li>Expose a {@link #reverse()} command for unjamming.
 *   <li>Stop wheels when the robot is disabled.
 * </ul>
 *
 * <h2>Speed Control Modes</h2>
 * {@link AimParams} can specify two types of output via {@link SpeedControl}:
 * <ul>
 *   <li>{@link SpeedControl#ProjectileVelocity} — the desired ball exit speed in m/s, converted
 *       internally to wheel RPS by scaling against {@link ShooterConstants#kMaxLinearSpeed}.
 *   <li>{@link SpeedControl#MechanismControl} — wheel speed in RPS directly (used by
 *       {@link frc.robot.aiming.ToFAim ToFAim} after calibration with the lookup table).
 * </ul>
 *
 * <h2>Recovery Mode</h2>
 * When a ball passes through the shooter, the wheels momentarily decelerate due to the ball's
 * inertia. Recovery mode increases motor output aggressively to pull wheel speed back up before
 * the next ball. It is enabled when the speed error exceeds
 * {@link ShooterConstants#kRecoveryErrorThreshold} — the same threshold used to detect a
 * "significant drop" in {@link #periodic()} for the {@link #seenBall(double)} trigger.
 *
 * <h2>IO Layer</h2>
 * All hardware calls go through {@link ShooterIO}:
 * <ul>
 *   <li>{@link ShooterIOHardware} — two TalonFX motors for wheels, one for hood.
 *   <li>{@link ShooterIOSim} — simulated motor physics.
 * </ul>
 * Mode is selected in {@link frc.robot.RobotStateMachine} at construction time.
 */
public class Shooter extends SubsystemBase {

  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  /** Current hood position reference (mechanism rotations). Updated each loop by {@link #shoot}. */
  private Angle hoodReference = Rotations.zero();

  /** Current wheel speed reference (RPS). Updated each loop by {@link #shoot}. */
  private AngularVelocity shooterReference = RotationsPerSecond.zero();

  /**
   * True while a {@link #shoot} command is running. Used by {@link #tracked} to prevent a false
   * "ready" signal when no shoot command is active (wheels might happen to be at zero, which
   * would match the reference, incorrectly signalling ready).
   */
  private boolean active = false;

  /**
   * WPILib timestamp of the most recent significant speed drop, used by {@link #seenBall(double)}
   * to detect ball passage through the shooter.
   */
  private double lastSignificantDrop = 0;

  /**
   * Constructs the shooter and registers disable-time cleanup and telemetry.
   *
   * <p>When the robot is disabled, wheel velocity is commanded to zero via an
   * {@code ignoringDisable} command so the wheels actually stop rather than coasting.
   *
   * @param io hardware or simulation implementation
   */
  public Shooter(ShooterIO io) {
    this.io = io;

    // Safety: when disabled, explicitly stop the wheels. Without this, the TalonFX would hold
    // its last output during disable transitions, keeping wheels spinning on the field.
    RobotModeTriggers.disabled().onTrue(
        this.runOnce(() -> io.setVelocity(RotationsPerSecond.zero())).ignoringDisable(true));

    OnboardLogger log = new OnboardLogger("Shooter");
    log.registerMeasurement("Hood Reference", () -> hoodReference, Rotations);
    log.registerMeasurement("Velocity Reference", () -> shooterReference, RotationsPerSecond);
    log.registerBoolean("Enable Recovery Mode", () -> shouldEnableRecovery(shooterReference));
  }

  /**
   * Reads hardware inputs and detects ball passage each loop.
   *
   * <p>A ball passing through the shooter causes a brief, sharp drop in wheel velocity. Tracking
   * this event supports the {@link #seenBall(double)} trigger used to sequence re-loading
   * (if implemented). The drop is detected by comparing actual vs reference speed against
   * {@link ShooterConstants#kShootingErrorDetectionThreshold}.
   */
  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Detect ball passage: if actual speed drops far below reference, a ball just went through.
    double error =
        shooterReference.baseUnitMagnitude() - inputs.shooter1Velocity.baseUnitMagnitude();
    if (error > ShooterConstants.kShootingErrorDetectionThreshold.baseUnitMagnitude()) {
      lastSignificantDrop = Timer.getTimestamp();
    }
  }

  /**
   * Converts a desired projectile exit speed to the required wheel angular velocity.
   *
   * <p>When {@code control == ProjectileVelocity}, the exit speed in m/s is scaled against the
   * maximum achievable ball speed ({@link ShooterConstants#kMaxLinearSpeed}) to get a fraction
   * of max wheel RPS ({@link ShooterConstants#kMaxRotationalSpeed}).
   *
   * <p>When {@code control == MechanismControl}, the value is taken as wheel RPS directly — used
   * by {@link frc.robot.aiming.ToFAim} which outputs calibrated wheel speeds.
   */
  private AngularVelocity projectileToShooterVelocity(double output, SpeedControl control) {
    switch (control) {
      case ProjectileVelocity:
        return ShooterConstants.kMaxRotationalSpeed
            .times(output / ShooterConstants.kMaxLinearSpeed.in(MetersPerSecond));
      case MechanismControl:
        return RotationsPerSecond.of(output);
    }
    return RotationsPerSecond.zero();
  }

  /**
   * Converts a desired launch pitch angle to the required hood mechanism angle.
   *
   * <p>The hood encoder reads 0 at some mechanical reference. The launch pitch is measured from
   * horizontal in the robot frame. Converting:
   * <pre>
   *   mechanismAngle = (90° − pitch) − HoodConstants.kOffset
   * </pre>
   * where the 90° subtraction is because the hood is measured from vertical (90° − pitch gives
   * degrees from vertical). {@link HoodConstants#kOffset} accounts for the encoder's zero not
   * being at the vertical reference.
   */
  private Angle pitchToHoodAngle(Rotation2d pitch) {
    Angle mechanismAngle =
        Rotation2d.kCCW_90deg.minus(pitch).getMeasure().minus(HoodConstants.kOffset);
    return mechanismAngle;
  }

  /**
   * Returns a command that continuously tracks the aim parameters and drives the shooter wheels
   * and hood toward the required setpoints.
   *
   * <h2>Loop Behavior</h2>
   * Each loop cycle (20 ms):
   * <ol>
   *   <li>Poll {@code paramsSupplier.get()} for the latest lead-compensated shot parameters.
   *   <li>If the shot is feasible (status == Possible), compute required wheel speed and hood
   *       angle and send them to the IO layer.
   *   <li>If infeasible (out of range or no target), command zero wheel speed so the wheels
   *       don't run without a valid target.
   * </ol>
   *
   * <h2>Why Wheels Stop on End (finallyDo)</h2>
   * When this command ends (trigger released, command interrupted, or auto step expires), the
   * {@code finallyDo} block runs: it sets {@code active = false} and commands wheel velocity to
   * zero. This prevents the wheels from spinning indefinitely after the driver releases the aim
   * button. If an immediate re-shot is needed, the next {@link #shoot} call will ramp the wheels
   * back up — the ramp time is minimized by the recovery mode logic.
   *
   * @param paramsSupplier called every loop to get the latest {@link AimParams} from the pipeline;
   *                       typically {@code state::aimParams} from {@link frc.robot.superstructure.StateManager}
   */
  public Command shoot(Supplier<AimParams> paramsSupplier) {
    return this.run(() -> {
      active = true;
      AimParams params = paramsSupplier.get();
      if (!params.isOk()) {
        // No valid shot — spin down to prevent unnecessary wear and energy use.
        io.setVelocity(RotationsPerSecond.zero(), false);
        return;
      }
      shooterReference = projectileToShooterVelocity(params.output, params.control);
      hoodReference = pitchToHoodAngle(params.pitch);
      io.setVelocity(shooterReference, shouldEnableRecovery(shooterReference));
      io.setAngle(hoodReference);
    })
        .finallyDo(() -> {
          active = false;
          shooterReference = RotationsPerSecond.zero();
          io.setVelocity(shooterReference);
        });
  }

  /**
   * Returns a command that spins the wheels in reverse at {@link ShooterConstants#kReverseVelocity}
   * to clear jams. Stops the wheels when the command ends.
   *
   * <p>Bound to the driver's right bumper in {@link frc.robot.RobotContainer}.
   */
  public Command reverse() {
    return this.startEnd(
        () -> io.setVelocity(ShooterConstants.kReverseVelocity),
        () -> io.setVelocity(RadiansPerSecond.zero()));
  }

  /**
   * Returns {@code true} when the shooter wheel speed is within {@link AimParams#deltaOutput} of
   * the target and the hood is within {@link AimParams#deltaPitch} of the target — meaning a shot
   * at this instant would be accurate.
   */
  private boolean shooterAtSpeed(AimParams params) {
    double velocityError = projectileToShooterVelocity(params.output, params.control)
        .minus(inputs.shooter1Velocity).baseUnitMagnitude();
    boolean velocityOk = velocityError <= params.deltaOutput;
    return velocityOk;
  }

  /**
   * Returns {@code true} when the speed error exceeds {@link ShooterConstants#kRecoveryErrorThreshold},
   * indicating the wheels need aggressive recovery (e.g., after a ball just cleared the shooter).
   */
  private boolean shouldEnableRecovery(AngularVelocity reference) {
    AngularVelocity error = reference.minus(inputs.shooter1Velocity);
    return error.baseUnitMagnitude() > ShooterConstants.kRecoveryErrorThreshold.baseUnitMagnitude();
  }

  /** Returns {@code true} when the hood angle is within {@link AimParams#deltaPitch} of target. */
  private boolean hoodAtPosition(AimParams params) {
    double hoodError =
        inputs.hoodPosition.minus(pitchToHoodAngle(params.pitch)).baseUnitMagnitude();
    boolean hoodOk =
        Math.abs(hoodError) <= params.deltaPitch.getMeasure().baseUnitMagnitude();
    return hoodOk;
  }

  /**
   * Returns a {@link Trigger} that is {@code true} when the shooter is both active (a
   * {@link #shoot} command is running) AND both the wheel speed and hood angle are within
   * tolerance of the current target parameters.
   *
   * <p>This is the primary "ready to fire" signal. It is polled by
   * {@link frc.robot.RobotStateMachine#isShootReady()} and gates the hopper in
   * {@link frc.robot.commands.ShootWhenReady}.
   *
   * <p>The {@code active} guard prevents a false positive when no shoot command is running
   * (wheels at zero, reference at zero → error at zero → incorrectly "ready").
   *
   * @param params supplier of current aim parameters; polled each scheduler tick
   */
  public Trigger tracked(Supplier<AimParams> params) {
    return new Trigger(() -> {
      if (!active) {
        return false;
      }
      AimParams realParams = params.get();
      return shooterAtSpeed(realParams) && hoodAtPosition(realParams);
    });
  }

  /**
   * A trigger that is {@code true} while a {@link #shoot} command is actively running (regardless
   * of whether the shot is within tolerance). Useful for LED feedback or auto-sequencing.
   */
  public Trigger shooting = new Trigger(() -> active);

  /**
   * Returns a trigger that fires when at least {@code seconds} have elapsed since the last
   * significant speed drop — indicating the ball has cleared the shooter and the wheels have
   * recovered.
   *
   * <p>Can be used to sequence a re-intake or to confirm ball passage for telemetry.
   *
   * @param seconds minimum time since last detected ball passage; 0.3–0.5 s is typical
   */
  public Trigger seenBall(double seconds) {
    return new Trigger(() -> {
      double timeSinceLastShot = Timer.getTimestamp() - lastSignificantDrop;
      return timeSinceLastShot > seconds;
    });
  }
}

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Revolutions;
import static edu.wpi.first.units.Units.RevolutionsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.OnboardLogger;

/**
 * Hardware abstraction interface for the turret mechanism.
 *
 * <h2>IO Layer Pattern</h2>
 * Rather than calling TalonFX or CANcoder directly, {@link frc.robot.subsystems.turret.Turret Turret}
 * calls this interface. Three implementations are selected at startup:
 * <ul>
 *   <li>{@link TurretIOHardware} — talks to real CTRE hardware over CAN.
 *   <li>{@link TurretIOSim} — drives simulated motor physics; matches hardware behavior well
 *       enough to test control loops in simulation without a robot.
 *   <li>{@link TurretIODisabled} — all calls are no-ops; safe for replay and unit tests.
 * </ul>
 * The pattern keeps all hardware-specific code in one place, making the subsystem itself
 * testable in isolation and safe to run during development.
 *
 * <h2>Contract</h2>
 * <ul>
 *   <li>Implementations must update {@code inputs} in-place inside {@link #updateInputs}; callers
 *       store the inputs object and read from it for the rest of the loop.
 *   <li>{@link #setPosition} commands the mechanism toward a target angle using MotionMagic
 *       (on hardware). It is the only output method — there is no voltage or percent-output mode.
 *   <li>{@link #calibrate} must be called when the turret is known to be at the home/limit-switch
 *       position. It zeros the CANcoder so subsequent position reads are field-valid.
 * </ul>
 */
public interface TurretIO {

  /**
   * Mutable snapshot of all turret sensor readings, refreshed each loop by
   * {@link #updateInputs(TurretIOInputs)}.
   *
   * <p>Fields are public and mutable by design — the IO implementation writes into this object
   * and the subsystem reads from it, avoiding allocations every loop.
   *
   * <p>All hardware telemetry is registered with {@link OnboardLogger} in the constructor so that
   * every field is automatically published to SmartDashboard / NT4 without additional {@code put*}
   * calls in {@code periodic()}.
   */
  public class TurretIOInputs {
    /** True when the TalonFX responds on CAN. False after a CAN fault or motor disconnect. */
    public boolean motorConnected = false;

    /**
     * True after {@link TurretIO#calibrate()} has been called and the encoder has been zeroed.
     * The turret should not track a field target until this is true.
     */
    public boolean calibrated = false;

    /** Motor supply voltage (battery-side, before internal resistance drop). */
    public Voltage voltage = Volts.zero();

    /** Supply current drawn from the battery (includes quiescent draw of the TalonFX). */
    public Current supplyCurrent = Amps.zero();

    /**
     * Stator (motor-side) current. Proportional to torque output.
     * High stator current with low velocity → near stall (mechanism hitting a hard stop).
     */
    public Current statorCurrent = Amps.zero();

    /**
     * Torque current — the component of stator current doing useful work (torque-producing).
     * Excludes magnetizing/reactive current. Useful for characterizing mechanism friction.
     */
    public Current torqueCurrent = Amps.zero();

    /** Motor winding temperature. TalonFX thermal limit is ~85 °C — log this to catch thermal issues. */
    public Temperature temperature = Celsius.zero();

    /** Mechanism angular velocity (revolutions per second of the turret ring, not the motor shaft). */
    public AngularVelocity velocity = RotationsPerSecond.zero();

    /**
     * Current MotionMagic position reference (what the controller is currently commanding toward).
     * In mechanism rotations. Comparing this to {@link #position} shows tracking error.
     */
    public Angle reference = Radians.zero();

    /** Actual mechanism position from the CANcoder absolute encoder, in radians / rotations. */
    public Angle position = Radians.zero();

    /**
     * Constructs the inputs object and registers all fields with the {@code "Turret"} logger.
     * Registration uses lambda suppliers so the logger always reads current values.
     */
    public TurretIOInputs() {
      OnboardLogger log = new OnboardLogger("Turret");
      log.registerBoolean("Calibrated", () -> calibrated);
      log.registerMeasurement("Voltage", () -> voltage, Volts);
      log.registerMeasurement("Supply Current", () -> supplyCurrent, Amps);
      log.registerMeasurement("Stator Current", () -> statorCurrent, Amps);
      log.registerMeasurement("Torque Current", () -> torqueCurrent, Amps);
      log.registerMeasurement("Temperature", () -> temperature, Celsius);
      log.registerMeasurement("Velocity", () -> velocity, RevolutionsPerSecond);
      log.registerMeasurement("Position", () -> position, Revolutions);
    }
  }

  /**
   * Reads the latest sensor values from hardware and writes them into {@code inputs}.
   * Called once per loop cycle by {@link frc.robot.subsystems.turret.Turret#periodic Turret.periodic()}.
   *
   * @param inputs mutable object to update in-place; the same instance is reused every loop
   */
  public void updateInputs(TurretIOInputs inputs);

  /**
   * Commands the turret toward the given mechanism angle using MotionMagic.
   * The implementation may call this as often as every loop cycle; MotionMagic handles
   * smooth acceleration and deceleration internally.
   *
   * @param position target mechanism angle (in any {@link edu.wpi.first.units.measure.Angle Angle}
   *                 unit — implementations convert to rotations internally)
   */
  public void setPosition(Angle position);

  /**
   * Zeros the encoder at the current physical position, declaring it the home reference.
   * Called once at startup when the turret is confirmed to be at the limit switch.
   * After this call, {@link TurretIOInputs#calibrated} should return {@code true}.
   */
  public void calibrate();
}

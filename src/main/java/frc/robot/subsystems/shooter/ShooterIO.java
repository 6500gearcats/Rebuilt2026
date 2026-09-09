package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.OnboardLogger;

/**
 * Hardware abstraction layer for the shooter mechanism.
 *
 * <p>The IO layer pattern separates <em>what</em> to do from <em>how</em> it is done on
 * hardware. Code that controls the shooter (e.g., {@link Shooter}) calls methods on this
 * interface without knowing whether it is running on a real TalonFX ({@link ShooterIOHardware}),
 * a physics simulation ({@link ShooterIOSim}), or a future replay harness.
 *
 * <p>Available implementations:
 * <ul>
 *   <li>{@link ShooterIOHardware} — real TalonFX motors via CTRE Phoenix 6 CAN.
 *   <li>{@link ShooterIOSim} — first-order lag simulation, no hardware required.
 * </ul>
 *
 * <p>The implementation is selected at construction time in {@link RobotStateMachine} based
 * on {@code Constants.RobotConstants.currentMode}.
 */
public interface ShooterIO {
  /**
   * Reads all hardware sensor values into {@code inputs}. Must be called once per loop cycle
   * before any other method so that callers see fresh data. All fields in {@link ShooterIOInputs}
   * are updated in place; the caller owns the inputs object across loops.
   */
  public void updateInputs(ShooterIOInputs inputs);

  /**
   * Snapshot of all shooter sensor readings for one loop cycle.
   *
   * <p>This class is a plain data holder — all fields are public and mutable. It also
   * self-registers every field with {@link OnboardLogger} in its constructor, so all
   * shooter hardware data is automatically written to the {@code .wpilog} file without
   * any additional setup in {@code Shooter.periodic()}.
   */
  class ShooterIOInputs {
    public boolean shooter1MotorConnected = false;
    public Current shooter1SupplyCurrent = Amps.zero();
    public Current shooter1TorqueCurrent = Amps.zero();
    public Current shooter1StatorCurrent = Amps.zero();
    public Voltage shooter1Voltage = Volts.zero();
    public Temperature shooter1Temperature = Celsius.zero();
    public AngularVelocity shooter1Velocity = RotationsPerSecond.zero();

    public boolean shooter2MotorConnected = false;
    public Current shooter2SupplyCurrent = Amps.zero();
    public Current shooter2TorqueCurrent = Amps.zero();
    public Current shooter2StatorCurrent = Amps.zero();
    public Voltage shooter2Voltage = Volts.zero();
    public Temperature shooter2Temperature = Celsius.zero();
    public AngularVelocity shooter2Velocity = RotationsPerSecond.zero();

    public boolean hoodMotorConnected = false;
    public Current hoodSupplyCurrent = Amps.zero();
    public Current hoodTorqueCurrent = Amps.zero();
    public Current hoodStatorCurrent = Amps.zero();
    public Voltage hoodVoltage = Volts.zero();
    public Temperature hoodTemperature = Celsius.zero();
    public AngularVelocity hoodVelocity = RotationsPerSecond.zero();
    public Angle hoodPosition = Radians.zero();
    public boolean hoodCANcoderConnected = false;
    public Angle hoodCANcoderPosition = Radians.zero();

    public ShooterIOInputs() {
      OnboardLogger log = new OnboardLogger("Shooter");
      log.registerBoolean("Shooter 1 Motor Connected", () -> shooter1MotorConnected);
      log.registerMeasurement("Shooter 1 Supply Current", () -> shooter1SupplyCurrent, Amps);
      log.registerMeasurement("Shooter 1 Torque Current", () -> shooter1TorqueCurrent, Amps);
      log.registerMeasurement("Shooter 1 Stator Current", () -> shooter1StatorCurrent, Amps);
      log.registerMeasurement("Shooter 1 Voltage", () -> shooter1Voltage, Volts);
      log.registerMeasurement("Shooter 1 Temperature", () -> shooter1Temperature, Celsius);
      log.registerMeasurement("Shooter 1 Velocity", () -> shooter1Velocity, RotationsPerSecond);
      log.registerEnergy("Shooter 1 Energy", () -> shooter1Voltage, () -> shooter1StatorCurrent);

      log.registerBoolean("Shooter 2 Motor Connected", () -> shooter2MotorConnected);
      log.registerMeasurement("Shooter 2 Supply Current", () -> shooter2SupplyCurrent, Amps);
      log.registerMeasurement("Shooter 2 Torque Current", () -> shooter2TorqueCurrent, Amps);
      log.registerMeasurement("Shooter 2 Stator Current", () -> shooter2StatorCurrent, Amps);
      log.registerMeasurement("Shooter 2 Voltage", () -> shooter2Voltage, Volts);
      log.registerMeasurement("Shooter 2 Temperature", () -> shooter2Temperature, Celsius);
      log.registerMeasurement("Shooter 2 Velocity", () -> shooter2Velocity, RotationsPerSecond);
      log.registerEnergy("Shooter 2 Energy", () -> shooter2Voltage, () -> shooter2StatorCurrent);

      log.registerBoolean("Hood Motor Connected", () -> hoodMotorConnected);
      log.registerMeasurement("Hood Supply Current", () -> hoodSupplyCurrent, Amps);
      log.registerMeasurement("Hood Torque Current", () -> hoodTorqueCurrent, Amps);
      log.registerMeasurement("Hood Stator Current", () -> hoodStatorCurrent, Amps);
      log.registerMeasurement("Hood Voltage", () -> hoodVoltage, Volts);
      log.registerMeasurement("Hood Temperature", () -> hoodTemperature, Celsius);
      log.registerMeasurement("Hood Velocity", () -> hoodVelocity, RotationsPerSecond);
      log.registerMeasurement("Hood Position", () -> hoodPosition, Rotations);
      log.registerEnergy("Hood Energy", () -> hoodVoltage, () -> hoodStatorCurrent);

      log.registerBoolean("Hood CANcoder Connected", () -> hoodCANcoderConnected);
      log.registerMeasurement("Hood CANcoder Position", () -> hoodCANcoderPosition, Rotations);
    }
  }

  /**
   * Commands the shooter flywheel to spin at {@code velocity}.
   *
   * <p>Two closed-loop modes are available, selected by {@code useRecovery}:
   * <ul>
   *   <li><b>Normal mode</b> ({@code useRecovery = false}): uses
   *       {@code VelocityTorqueCurrentFOC} — torque-based FOC control. Provides excellent
   *       speed stability and recovers quickly after a note passes through.
   *       This is the default for all match shooting.
   *   <li><b>Recovery mode</b> ({@code useRecovery = true}): uses
   *       {@code VelocityDutyCycle} — voltage-based control. Less accurate but draws less
   *       current during brown-out conditions or when the battery is depleted. Use if the
   *       robot is tripping breakers or experiencing voltage sag.
   * </ul>
   * Passing zero velocity coasts the motor instead of actively braking.
   *
   * @param velocity    Target flywheel speed.
   * @param useRecovery {@code true} to switch to the lower-current recovery control mode.
   */
  public void setVelocity(AngularVelocity velocity, boolean useRecovery);

  /** Convenience overload — equivalent to {@code setVelocity(velocity, false)}. */
  public default void setVelocity(AngularVelocity velocity) {
    setVelocity(velocity, false);
  }

  /**
   * Commands the hood to move to {@code angle}.
   * The hood adjusts the vertical launch angle of the note.
   *
   * @param angle Target hood position (encoder rotations from the home/flat position).
   */
  public void setAngle(Angle angle);
}

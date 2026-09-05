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

public interface TurretIO {

  public class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean calibrated = false;
    public Voltage voltage = Volts.zero();
    public Current supplyCurrent = Amps.zero();
    public Current statorCurrent = Amps.zero();
    public Current torqueCurrent = Amps.zero();
    public Temperature temperature = Celsius.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public Angle reference = Radians.zero();
    public Angle position = Radians.zero();

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

  public void updateInputs(TurretIOInputs inputs);

  public void setPosition(Angle position);

  public void calibrate();
}

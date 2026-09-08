package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Simulation implementation of {@link TurretIO}. Models the turret as a first-order lag
 * system (20% blend per loop = ~3 loop cycles to get within 0.1% of target), which is
 * fast enough that turret-tracking behavior in simulation is visually realistic.
 *
 * <p>Calibration state is read from SmartDashboard so testers can toggle
 * {@code "Turret/Successful Calibration?"} in the dashboard without redeploying code.
 * This simulates the physical limit-switch hit that triggers calibration on hardware.
 */
public class TurretIOSim implements TurretIO {
  private Angle position = Radians.zero();
  private Angle reference = Radians.zero();

  private final String calibrationLabel = "Turret/Successful Calibration?";

  /**
   * Advances the simulated turret position by one step toward the reference, then
   * updates all inputs. The calibration flag is read from SmartDashboard so tests can
   * toggle it without code changes.
   */
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;
    position = position.times(0.8).plus(reference.times(0.2));
    inputs.position = position;
    inputs.reference = reference;
    inputs.calibrated = SmartDashboard.getBoolean(calibrationLabel, true);
  }

  /**
   * Sets the target position. The simulated turret will move toward this reference
   * over subsequent {@link #updateInputs} calls.
   *
   * @param position Target angle in the turret's encoder frame.
   */
  public void setPosition(Angle position) {
    reference = position;
  }

  /** No-op in simulation — calibration is read from SmartDashboard instead. */
  public void calibrate() {}
}

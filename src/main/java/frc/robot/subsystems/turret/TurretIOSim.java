package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretIOSim implements TurretIO {
  private Angle position = Radians.zero();
  private Angle reference = Radians.zero();

  private final String calibrationLabel = "Turret/Successful Calibration?";

  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;
    position = position.times(0.8).plus(reference.times(0.2));
    inputs.position = position;
    inputs.reference = reference;
    inputs.calibrated = SmartDashboard.getBoolean(calibrationLabel, true);
  }

  public void setPosition(Angle position) {
    reference = position;
  }

  public void calibrate() {}
}

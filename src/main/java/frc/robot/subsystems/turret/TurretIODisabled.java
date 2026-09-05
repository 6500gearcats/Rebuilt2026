package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;

public class TurretIODisabled implements TurretIO {
  public void calibrate() {}

  public void setPosition(Angle angle) {}

  public void updateInputs(TurretIOInputs inputs) {}
}

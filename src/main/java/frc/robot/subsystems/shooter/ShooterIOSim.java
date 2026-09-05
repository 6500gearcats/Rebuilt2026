package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public class ShooterIOSim implements ShooterIO {

  private Angle hoodAngle = Rotations.zero();
  private AngularVelocity shooterVelocity = RotationsPerSecond.zero();

  public ShooterIOSim() {}

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooter1MotorConnected = true;
    inputs.shooter2MotorConnected = true;
    inputs.shooter1Velocity = shooterVelocity;
    inputs.shooter2Velocity = shooterVelocity;

    inputs.hoodMotorConnected = true;
    inputs.hoodPosition = hoodAngle;
  }

  public void setVelocity(AngularVelocity velocity, boolean useRecovery) {
    shooterVelocity = velocity;
  }

  public void setAngle(Angle angle) {
    hoodAngle = angle;
  }
}

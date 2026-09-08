package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Simulation implementation of {@link ShooterIO}. Models the flywheel as a first-order lag
 * system so that the {@link Shooter} subsystem's {@code isUpToSpeed()} logic works realistically
 * in simulation without any physical hardware.
 *
 * <p>The hood position is set instantly (no lag) because hood dynamics are not important to
 * test in simulation at this stage.
 */
public class ShooterIOSim implements ShooterIO {

  // Commanded target — what the control loop wants.
  private AngularVelocity targetVelocity = RotationsPerSecond.zero();

  // Simulated actual speed — approaches target via a first-order lag each 20 ms loop.
  // Alpha 0.94 / blend 0.06 gives ~1 s to reach 95% of target, realistic for a Falcon
  // 500 driving a flywheel with moderate rotational inertia.
  private AngularVelocity actualVelocity = RotationsPerSecond.zero();
  private static final double kAlpha = 0.94;

  private Angle hoodAngle = Rotations.zero();

  public ShooterIOSim() {}

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Apply first-order lag: step actual velocity toward target each loop.
    actualVelocity = actualVelocity.times(kAlpha).plus(targetVelocity.times(1.0 - kAlpha));

    inputs.shooter1MotorConnected = true;
    inputs.shooter2MotorConnected = true;
    inputs.shooter1Velocity = actualVelocity;
    inputs.shooter2Velocity = actualVelocity;

    inputs.hoodMotorConnected = true;
    inputs.hoodPosition = hoodAngle;
  }

  @Override
  public void setVelocity(AngularVelocity velocity, boolean useRecovery) {
    targetVelocity = velocity;
  }

  @Override
  public void setAngle(Angle angle) {
    hoodAngle = angle;
  }
}

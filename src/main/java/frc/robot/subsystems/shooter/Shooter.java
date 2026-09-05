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

/** Shooter subsystem. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputs inputs = new ShooterIOInputs();

  private Angle hoodReference = Rotations.zero();
  private AngularVelocity shooterReference = RotationsPerSecond.zero();

  private boolean active = false;

  private double lastSignificantDrop = 0;

  public Shooter(ShooterIO io) {
    this.io = io;

    RobotModeTriggers.disabled().onTrue(
        this.runOnce(() -> io.setVelocity(RotationsPerSecond.zero())).ignoringDisable(true));

    OnboardLogger log = new OnboardLogger("Shooter");
    log.registerMeasurement("Hood Reference", () -> hoodReference, Rotations);
    log.registerMeasurement("Velocity Reference", () -> shooterReference, RotationsPerSecond);
    log.registerBoolean("Enable Recovery Mode", () -> shouldEnableRecovery(shooterReference));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    double error =
        shooterReference.baseUnitMagnitude() - inputs.shooter1Velocity.baseUnitMagnitude();
    if (error > ShooterConstants.kShootingErrorDetectionThreshold.baseUnitMagnitude()) {
      lastSignificantDrop = Timer.getTimestamp();
    }
  }

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

  private Angle pitchToHoodAngle(Rotation2d pitch) {
    Angle mechanismAngle =
        Rotation2d.kCCW_90deg.minus(pitch).getMeasure().minus(HoodConstants.kOffset);
    return mechanismAngle;
  }

  /**
   * This command tells the shooter to begin to track the desired target. When this command ends, it
   * does NOT turn off the shooter; we don't want to have to get all the way back up to speed.
   */
  public Command shoot(Supplier<AimParams> paramsSupplier) {
    return this.run(() -> {
      active = true;
      AimParams params = paramsSupplier.get();
      if (!params.isOk()) {
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

  /** Runs the shooter in reverse, to potentially remove any jams. */
  public Command reverse() {
    return this.startEnd(
        () -> io.setVelocity(ShooterConstants.kReverseVelocity),
        () -> io.setVelocity(RadiansPerSecond.zero()));
  }

  private boolean shooterAtSpeed(AimParams params) {
    double velocityError = projectileToShooterVelocity(params.output, params.control)
        .minus(inputs.shooter1Velocity).baseUnitMagnitude();
    boolean velocityOk = velocityError <= params.deltaOutput;
    return velocityOk;
  }

  private boolean shouldEnableRecovery(AngularVelocity reference) {
    AngularVelocity error = reference.minus(inputs.shooter1Velocity);
    return error.baseUnitMagnitude() > ShooterConstants.kRecoveryErrorThreshold.baseUnitMagnitude();
  }

  private boolean hoodAtPosition(AimParams params) {
    double hoodError =
        inputs.hoodPosition.minus(pitchToHoodAngle(params.pitch)).baseUnitMagnitude();
    boolean hoodOk =
        Math.abs(hoodError) <= params.deltaPitch.getMeasure().baseUnitMagnitude();
    return hoodOk;
  }

  public Trigger tracked(Supplier<AimParams> params) {
    return new Trigger(() -> {
      if (!active) {
        return false;
      }

      AimParams realParams = params.get();

      return shooterAtSpeed(realParams) && hoodAtPosition(realParams);
    });
  }

  public Trigger shooting = new Trigger(() -> active);

  public Trigger seenBall(double seconds) {
    return new Trigger(() -> {
      double timeSinceLastShot = Timer.getTimestamp() - lastSignificantDrop;
      return timeSinceLastShot > seconds;
    });
  }
}

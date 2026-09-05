package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.aiming.AimParams;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;
import frc.robot.superstructure.StateManager;
import frc.robot.util.OnboardLogger;
import frc.robot.vision.localization.LocalizationConstants;

public class Turret extends SubsystemBase {

  private final TurretIO io;
  private final TurretIOInputs inputs;

  private final Alert calibrationAlert =
      new Alert("Turret not calibrated successfully", AlertType.kError);

  private boolean tracking;

  private Angle reference = TurretConstants.kHomePosition;

  public Turret(TurretIO io) {
    super();
    this.io = io;
    inputs = new TurretIOInputs();
    io.calibrate();
    SmartDashboard.putData("Turret/Home", home());
    Command calibrate = runOnce(io::calibrate).ignoringDisable(true);
    SmartDashboard.putData("Turret/Calibrate", calibrate);
    RobotModeTriggers.disabled().onTrue(calibrate);

    OnboardLogger log = new OnboardLogger("Turret");
    log.registerBoolean("Ready", ready());
    log.registerBoolean("Tracking", () -> tracking);
    log.registerMeasurement("Reference", () -> reference, Rotations);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    calibrationAlert.set(!inputs.calibrated);
  }

  /**
   * Continually tracks the aim target from the {@link StateManager}, accounting for robot rotation.
   */
  public Command track(StateManager state) {
    return Commands.sequence(
        this.run(() -> {
          tracking = true;
          Rotation2d robot = state.robotPose().getRotation();
          AimParams params = state.aimParams();
          if (!params.isOk()) {
            return;
          }
          Angle mechanismAngle = params.yaw.minus(robot).getMeasure().plus(TurretConstants.kForwards);
          setPosition(mechanismAngle, !state.shootReady.getAsBoolean());
        }))
        .finallyDo(() -> tracking = false);
  }

  /**
   * Sends the turret to its home position and waits until it arrives.
   */
  public Command home() {
    return Commands.sequence(
        runOnce(() -> setPosition(TurretConstants.kHomePosition, false)),
        Commands.waitUntil(ready()));
  }

  public Command forwards() {
    return Commands.sequence(
        runOnce(() -> setPosition(TurretConstants.kForwards, false)),
        Commands.waitUntil(ready()));
  }

  /**
   * Trigger that is true while the turret is at its reference position.
   */
  public Trigger ready() {
    return new Trigger(() -> {
      double delta = inputs.position.minus(inputs.reference).baseUnitMagnitude();
      return Math.abs(delta) <= TurretConstants.kTolerance.baseUnitMagnitude();
    });
  }

  public Trigger tracked(Supplier<AimParams> params) {
    return new Trigger(() -> {
      double delta = inputs.position.minus(inputs.reference).baseUnitMagnitude();
      double epsilon = params.get().deltaYaw.getMeasure().baseUnitMagnitude();
      return Math.abs(delta) <= epsilon && tracking;
    });
  }

  public Pose3d turretPose(Pose2d robotPose) {
    return new Pose3d(robotPose).transformBy(TurretConstants.kOffset);
  }

  /** Calculates the "ideal" position for the turret, choosing the closest congruent turn. */
  private void setPosition(Angle position, boolean tracking) {
    Angle min = TurretConstants.kMinAngle;
    Angle max = TurretConstants.kMaxAngle;
    reference = Rotations.of(findCC(
        inputs.position.in(Rotations),
        position.in(Rotations),
        min.in(Rotations),
        max.in(Rotations)));
    io.setPosition(reference);
  }

  /**
   * Returns the Closest Congruent value in the range [min, max], modulo 1.
   *
   * @param position  current non-wrapped position
   * @param reference goal position in [0, 1]
   * @param min       minimum position
   * @param max       maximum position
   */
  protected static double findCC(double position, double reference, double min, double max) {
    while (reference > max) reference -= 1.0;
    while (reference < min) reference += 1.0;

    if (reference > max) return max;

    double error = Math.abs(reference - position);
    if (error < 0.5) return reference;

    double offset = (reference > position) ? -1 : 1;
    while (true) {
      double newReference = reference + offset;
      if (newReference > max || newReference < min) break;
      double newError = Math.abs(newReference - position);
      if (Math.abs(newError) < 0.5) return newReference;
      reference = newReference;
    }
    return reference;
  }

  /**
   * Jogs the turret at a fixed speed, stepping the position reference each loop.
   * Stops when the command ends. Typical speed: ±0.5 rot/s.
   */
  public Command jog(double rotationsPerSecond) {
    return this.run(() -> setPosition(reference.plus(Rotations.of(rotationsPerSecond * 0.02)), false));
  }

  public Transform3d turretCameraOffset() {
    Transform3d turretRelative =
        new Transform3d(Translation3d.kZero,
            new Rotation3d(new Rotation2d(inputs.position.minus(TurretConstants.kForwards))))
            .plus(LocalizationConstants.kTurretAoRToTurretCameraOffset);
    return TurretConstants.kOffset.plus(turretRelative);
  }
}

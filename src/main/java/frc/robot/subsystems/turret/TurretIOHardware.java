package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.StatusSignalUtil;

/**
 * Hardware implementation of {@link TurretIO} using a single absolute CANcoder for position
 * feedback and a TalonFX for closed-loop position control.
 *
 * <h2>Why a single CANcoder?</h2>
 * A previous design (Hackbots) used two encoders (a CANcoder for initial calibration and the
 * motor's internal encoder for running position). This class uses only the absolute CANcoder,
 * which avoids accumulated drift but requires the CANcoder to be reliable across the match.
 * The motor encoder is synced to the CANcoder once at calibration time via {@link #calibrate()}.
 * After that, the motor's internal encoder is used for position feedback, which is faster to read
 * and avoids CAN latency on every loop cycle.
 *
 * <h2>Motion control</h2>
 * Position setpoints are executed with {@code DynamicMotionMagicVoltage}, which generates an
 * S-curve velocity profile between the current and target positions. "Dynamic" means the cruise
 * velocity and acceleration can be set per-request, allowing the turret to move faster for large
 * slews and slower for fine adjustments (though currently using fixed values from
 * {@link TurretConstants}).
 *
 * <h2>Calibration</h2>
 * The turret must be calibrated before use (see {@link #calibrate()}). On hardware, calibration
 * is triggered when the turret hits its limit switch. Until then, {@code inputs.calibrated}
 * remains {@code false} and the {@link frc.robot.subsystems.turret.Turret} subsystem will
 * not accept tracking commands.
 */
public class TurretIOHardware implements TurretIO {

  private final TalonFX motor;
  private final CANcoder encoder;

  private final DynamicMotionMagicVoltage control;

  private Angle reference = Radians.zero();
  private boolean calibrated = false;

  /**
   * Constructs the hardware layer. Applies motor and encoder configs from
   * {@link TurretConstants} and registers all status signals for bulk refresh.
   * CAN IDs and encoder offset are set in Stage 8.
   */
  public TurretIOHardware() {
    motor = new TalonFX(TurretConstants.kMotorId);
    motor.getConfigurator().apply(TurretConstants.kMotorConfig);

    encoder = new CANcoder(TurretConstants.kEncoderId);
    encoder.getConfigurator().apply(TurretConstants.kEncoderConfig);

    control = new DynamicMotionMagicVoltage(
        0,
        TurretConstants.kMaxSpeed,
        TurretConstants.kMaxAcceleration);

    StatusSignalUtil.registerRioSignals(
        motor.getMotorVoltage(false),
        motor.getSupplyCurrent(false),
        motor.getStatorCurrent(false),
        motor.getTorqueCurrent(false),
        motor.getDeviceTemp(false),
        motor.getVelocity(false),
        motor.getPosition(false),
        encoder.getAbsolutePosition(false));
  }

  /**
   * Commands the turret to move to {@code reference} using DynamicMotionMagic.
   * The setpoint is also cached so {@link #updateInputs} can echo it back for telemetry.
   *
   * @param reference Target angle in the motor encoder frame (rotations, with the offset
   *                  from {@link TurretConstants#kForwards} already applied by the caller).
   */
  public void setPosition(Angle reference) {
    this.reference = reference;
    motor.setControl(control.withPosition(reference));
  }

  /**
   * Reads all hardware signals into {@code inputs}.
   *
   * <p>After {@link #calibrate()} is called, the motor encoder tracks position; the absolute
   * CANcoder position is no longer read at runtime because the motor's internal sensor is
   * faster and latency-free compared to reading the CANcoder over CAN every loop.
   */
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = BaseStatusSignal.isAllGood(
        motor.getMotorVoltage(false),
        motor.getSupplyCurrent(false),
        motor.getStatorCurrent(false),
        motor.getTorqueCurrent(false),
        motor.getDeviceTemp(false),
        motor.getVelocity(false),
        motor.getPosition(false));
    inputs.calibrated = calibrated;
    inputs.voltage = motor.getMotorVoltage(false).getValue();
    inputs.supplyCurrent = motor.getSupplyCurrent(false).getValue();
    inputs.statorCurrent = motor.getStatorCurrent(false).getValue();
    inputs.torqueCurrent = motor.getTorqueCurrent(false).getValue();
    inputs.temperature = motor.getDeviceTemp(false).getValue();
    inputs.velocity = motor.getVelocity(false).getValue();
    inputs.position = motor.getPosition(false).getValue();
    inputs.reference = reference;
  }

  /**
   * Calibrates the turret by seeding the motor's internal encoder with the CANcoder's
   * absolute position. Called once when the turret hits its home limit switch.
   *
   * <p>After this call, all subsequent position feedback comes from the motor encoder
   * (via {@link #updateInputs}), which is faster to read than the CANcoder.
   */
  public void calibrate() {
    Angle absPosition = encoder.getAbsolutePosition().getValue();
    motor.setPosition(absPosition);
    calibrated = true;
  }
}

package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Radians;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import frc.robot.util.StatusSignalUtil;

/**
 * Single-encoder turret hardware implementation. Gearcats uses one absolute CANcoder;
 * the Hackbots CRT (two-encoder) logic has been removed (4-4).
 * Encoder offset set in Stage 8-3.
 */
public class TurretIOHardware implements TurretIO {

  private final TalonFX motor;
  private final CANcoder encoder;

  private final DynamicMotionMagicVoltage control;

  private Angle reference = Radians.zero();
  private boolean calibrated = false;

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

  public void setPosition(Angle reference) {
    this.reference = reference;
    motor.setControl(control.withPosition(reference));
  }

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

  public void calibrate() {
    // Read absolute encoder position and seed the motor's internal sensor
    Angle absPosition = encoder.getAbsolutePosition().getValue();
    motor.setPosition(absPosition);
    calibrated = true;
  }
}

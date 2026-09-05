package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.util.StatusSignalUtil;

public class ShooterIOHardware implements ShooterIO {
  private final TalonFX shooter1Motor;
  private final TalonFX shooter2Motor;

  private final TalonFX hoodMotor;
  private final CANcoder hoodCANcoder;

  private AngularVelocity lastVelocity = RotationsPerSecond.zero();
  private boolean lastRecoveryEnabled = false;
  private Angle lastAngle = Rotations.zero();

  private final VelocityTorqueCurrentFOC shooterControl = new VelocityTorqueCurrentFOC(0);
  private final VelocityDutyCycle speedupControl = new VelocityDutyCycle(0);
  private final CoastOut coastControl = new CoastOut();

  private final DynamicMotionMagicVoltage hoodControl =
      new DynamicMotionMagicVoltage(0, 0, 0).withSlot(ShooterConstants.HoodConstants.kSlot);

  public ShooterIOHardware() {
    shooter1Motor = new TalonFX(ShooterConstants.kMotor1Id);
    shooter1Motor.getConfigurator().apply(ShooterConstants.kMotorConfig);

    shooter2Motor = new TalonFX(ShooterConstants.kMotor2Id);
    shooter2Motor.getConfigurator().apply(ShooterConstants.kMotorConfig);
    shooter2Motor
        .setControl(new Follower(ShooterConstants.kMotor1Id, ShooterConstants.kMotor2Alignment));

    hoodMotor = new TalonFX(HoodConstants.kMotorID);
    hoodMotor.getConfigurator().apply(HoodConstants.kMotorConfig);

    hoodCANcoder = new CANcoder(HoodConstants.kCANcoderId);
    hoodCANcoder.getConfigurator().apply(HoodConstants.kCANcoderConfig);

    StatusSignalUtil.registerRioSignals(
        shooter1Motor.getSupplyCurrent(false),
        shooter1Motor.getTorqueCurrent(false),
        shooter1Motor.getStatorCurrent(false),
        shooter1Motor.getMotorVoltage(false),
        shooter1Motor.getDeviceTemp(false),
        shooter1Motor.getVelocity(false),

        shooter2Motor.getSupplyCurrent(false),
        shooter2Motor.getTorqueCurrent(false),
        shooter2Motor.getStatorCurrent(false),
        shooter2Motor.getMotorVoltage(false),
        shooter2Motor.getDeviceTemp(false),
        shooter2Motor.getVelocity(false),

        hoodMotor.getSupplyCurrent(false),
        hoodMotor.getTorqueCurrent(false),
        hoodMotor.getStatorCurrent(false),
        hoodMotor.getMotorVoltage(false),
        hoodMotor.getDeviceTemp(false),
        hoodMotor.getVelocity(false),
        hoodMotor.getPosition(false),

        hoodCANcoder.getPosition(false));
  }

  public void updateInputs(ShooterIOInputs inputs) {
    inputs.shooter1MotorConnected = BaseStatusSignal.isAllGood(
        shooter1Motor.getSupplyCurrent(false),
        shooter1Motor.getTorqueCurrent(false),
        shooter1Motor.getStatorCurrent(false),
        shooter1Motor.getMotorVoltage(false),
        shooter1Motor.getDeviceTemp(false),
        shooter1Motor.getVelocity(false));

    inputs.shooter1SupplyCurrent = shooter1Motor.getSupplyCurrent(false).getValue();
    inputs.shooter1TorqueCurrent = shooter1Motor.getTorqueCurrent(false).getValue();
    inputs.shooter1StatorCurrent = shooter1Motor.getStatorCurrent(false).getValue();
    inputs.shooter1Voltage = shooter1Motor.getMotorVoltage(false).getValue();
    inputs.shooter1Temperature = shooter1Motor.getDeviceTemp(false).getValue();
    inputs.shooter1Velocity = shooter1Motor.getVelocity(false).getValue();

    inputs.shooter2MotorConnected = BaseStatusSignal.isAllGood(
        shooter2Motor.getSupplyCurrent(false),
        shooter2Motor.getTorqueCurrent(false),
        shooter2Motor.getStatorCurrent(false),
        shooter2Motor.getMotorVoltage(false),
        shooter2Motor.getDeviceTemp(false),
        shooter2Motor.getVelocity(false));

    inputs.shooter2SupplyCurrent = shooter2Motor.getSupplyCurrent(false).getValue();
    inputs.shooter2TorqueCurrent = shooter2Motor.getTorqueCurrent(false).getValue();
    inputs.shooter2StatorCurrent = shooter2Motor.getStatorCurrent(false).getValue();
    inputs.shooter2Voltage = shooter2Motor.getMotorVoltage(false).getValue();
    inputs.shooter2Temperature = shooter2Motor.getDeviceTemp(false).getValue();
    inputs.shooter2Velocity = shooter2Motor.getVelocity(false).getValue();

    inputs.hoodMotorConnected = BaseStatusSignal.isAllGood(
        hoodMotor.getSupplyCurrent(false),
        hoodMotor.getTorqueCurrent(false),
        hoodMotor.getStatorCurrent(false),
        hoodMotor.getMotorVoltage(false),
        hoodMotor.getDeviceTemp(false),
        hoodMotor.getVelocity(false),
        hoodMotor.getPosition(false));

    inputs.hoodSupplyCurrent = hoodMotor.getSupplyCurrent(false).getValue();
    inputs.hoodTorqueCurrent = hoodMotor.getTorqueCurrent(false).getValue();
    inputs.hoodStatorCurrent = hoodMotor.getStatorCurrent(false).getValue();
    inputs.hoodVoltage = hoodMotor.getMotorVoltage(false).getValue();
    inputs.hoodTemperature = hoodMotor.getDeviceTemp(false).getValue();
    inputs.hoodVelocity = hoodMotor.getVelocity(false).getValue();
    inputs.hoodPosition = hoodMotor.getPosition(false).getValue();

    inputs.hoodCANcoderConnected = BaseStatusSignal.isAllGood(
        hoodCANcoder.getPosition(false));
    inputs.hoodCANcoderPosition = hoodCANcoder.getPosition(false).getValue();
  }

  public void setVelocity(AngularVelocity velocity, boolean useRecovery) {
    if (velocity.baseUnitMagnitude() == 0) {
      shooter1Motor.setControl(coastControl);
      return;
    }
    if (!velocity.equals(lastVelocity) || useRecovery != lastRecoveryEnabled) {
      if (!useRecovery) {
        shooter1Motor.setControl(shooterControl.withVelocity(velocity));
      } else {
        shooter1Motor.setControl(speedupControl.withVelocity(velocity));
      }
      lastVelocity = velocity;
      lastRecoveryEnabled = useRecovery;
    }
  }

  public void setAngle(Angle angle) {
    if (!angle.equals(lastAngle)) {
      hoodMotor.setControl(hoodControl.withPosition(angle));
      lastAngle = angle;
    }
  }
}

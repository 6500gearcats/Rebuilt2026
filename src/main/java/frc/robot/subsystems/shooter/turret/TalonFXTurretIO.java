package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class TalonFXTurretIO implements TurretIO {
    private final TalonFX motor;
    private final TalonFXConfiguration talonFXConfigs;
    private PositionVoltage request;

    public TalonFXTurretIO() {
        motor = new TalonFX(Constants.MotorConstants.kTurretYawMotorID);
        request = new PositionVoltage(0).withSlot(2);

        talonFXConfigs = new TalonFXConfiguration();
        configureSlots();
        motor.getConfigurator().apply(talonFXConfigs);
    }

    private void configureSlots() {
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.2;
        slot0Configs.kV = 5;
        slot0Configs.kA = 3;
        slot0Configs.kP = 3;
        slot0Configs.kI = 0;
        slot0Configs.kD = 0.4;

        var slot1Configs = talonFXConfigs.Slot1;
        slot1Configs.kS = 0.2;
        slot1Configs.kV = SmartDashboard.getNumber("kV", 0);
        slot1Configs.kA = SmartDashboard.getNumber("kA", 0);
        slot1Configs.kP = SmartDashboard.getNumber("kP", 0);
        slot1Configs.kI = 0;
        slot1Configs.kD = SmartDashboard.getNumber("kD", 0);

        var slot2Configs = talonFXConfigs.Slot2;
        slot2Configs.kS = 0.2;
        slot2Configs.kV = 13;
        slot2Configs.kA = 5;
        slot2Configs.kP = 6;
        slot2Configs.kI = 0;
        slot2Configs.kD = 1;
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public double getSpeed() {
        return motor.get();
    }

    @Override
    public void setPosition(double rot) {
        motor.setControl(request.withPosition(rot));
    }

    @Override
    public double getMotorPosition() {
        return motor.getPosition().getValueAsDouble();
    }

    @Override
    public void zeroPosition() {
        motor.setPosition(0);
    }

    @Override
    public void updateSlotConfigs() {
        var slot = talonFXConfigs.Slot1;
        slot.kV = SmartDashboard.getNumber("kV", 0);
        slot.kA = SmartDashboard.getNumber("kA", 0);
        slot.kP = SmartDashboard.getNumber("kP", 0);
        slot.kI = SmartDashboard.getNumber("kI", 0);
        slot.kD = SmartDashboard.getNumber("kD", 0);
        request = new PositionVoltage(0).withSlot(1);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorPositionRotations = motor.getPosition().getValueAsDouble();
        inputs.motorVelocityRps = motor.getVelocity().getValueAsDouble();
        inputs.motorAppliedOutput = motor.get();
    }
}

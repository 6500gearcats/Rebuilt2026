package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants.MotorConstants;

public class TalonFXIntakeIO implements IntakeIO {
    TalonFX m_deployMotor;
    TalonFX m_intakeMotor;

    public TalonFXIntakeIO() {
        m_deployMotor = new TalonFX(MotorConstants.kIntakeDeployMotorID);
        m_intakeMotor = new TalonFX(MotorConstants.kIntakeMotorID);
    }

    @Override
    public void setDeploySpeed(double speed) {
        m_deployMotor.set(speed);
    }

    @Override
    public void setIntakeSpeed(double speed) {
        m_intakeMotor.set(speed);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.deploySpeed = m_deployMotor.get();
        inputs.intakeSpeed = m_intakeMotor.get();
    }

}

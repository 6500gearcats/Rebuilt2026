package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants.MotorConstants;

public class SparkIntakeIO implements IntakeIO {
    SparkMax m_deployMotor;
    SparkMax m_intakeMotor;

    public SparkIntakeIO() {
        m_deployMotor = new SparkMax(MotorConstants.kIntakeDeployMotorID, MotorType.kBrushless);
        m_intakeMotor = new SparkMax(MotorConstants.kIntakeMotorID, MotorType.kBrushless);
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

package frc.robot.subsystems.hopper.indexer;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class TalonFXIndexerIO implements IndexerIO {
    private TalonFX m_hopperMotor;

    public TalonFXIndexerIO() {
        m_hopperMotor = new TalonFX(Constants.MotorConstants.kIndexerMotorID);
    }

    @Override
    public void setSpeed(double speed) {
        m_hopperMotor.set(speed);
    }

    @Override
    public double getSpeed() {
        return m_hopperMotor.get();
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.speed = m_hopperMotor.get();
    }
}

package frc.robot.subsystems.hopper.indexer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

public class SparkIndexerIO implements IndexerIO {
    private Spark m_hopperMotor;

    public SparkIndexerIO() {
        m_hopperMotor = new Spark(Constants.MotorConstants.kIndexerMotorID);
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

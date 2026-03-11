package frc.robot.subsystems.hopper.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog
    class IndexerIOInputs {
        double speed;
    }

    public void setSpeed(double speed);

    public double getSpeed();

    public default void updateInputs(IndexerIOInputs inputs) {
    }
}

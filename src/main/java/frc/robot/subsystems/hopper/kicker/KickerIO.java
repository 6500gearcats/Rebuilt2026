package frc.robot.subsystems.hopper.kicker;

import org.littletonrobotics.junction.AutoLog;

public interface KickerIO {
    @AutoLog
    class KickerIOInputs {
        double speed;
    }

    public void setSpeed(double speed);

    public double getSpeed();

    public default void updateInputs(KickerIOInputs inputs) {
    }
}

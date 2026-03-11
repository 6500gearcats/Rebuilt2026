package frc.robot.subsystems.shooter.flywheel;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {
    @AutoLog
    class FlywheelIOInputs {
        double rightSpeed;
        double leftSpeed;
    }

    public void setSpeed(double speed);

    public double getSpeed();

    public default void updateInputs(FlywheelIOInputs inputs) {

    }
}

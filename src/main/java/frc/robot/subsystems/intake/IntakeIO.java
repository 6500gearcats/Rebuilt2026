package frc.robot.subsystems.intake;

import java.util.ArrayList;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        double deploySpeed;
        double intakeSpeed;
    }

    public void setDeploySpeed(double speed);

    public void setIntakeSpeed(double speed);

    public default void updateInputs(IntakeIOInputs inputs) {
    }
}

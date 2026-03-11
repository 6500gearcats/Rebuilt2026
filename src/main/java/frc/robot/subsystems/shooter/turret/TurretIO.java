package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
    @AutoLog
    class TurretIOInputs {
        double motorPositionRotations;
        double motorVelocityRps;
        double motorAppliedOutput;
    }

    void setSpeed(double speed);

    void setPosition(double rot);

    double getMotorPosition();

    double getSpeed();

    void zeroPosition();

    void updateSlotConfigs();

    default void updateInputs(TurretIOInputs inputs) {
    }
}

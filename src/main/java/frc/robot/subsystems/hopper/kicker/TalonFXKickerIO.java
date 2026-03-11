package frc.robot.subsystems.hopper.kicker;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;

public class TalonFXKickerIO implements KickerIO {
    // * Make sure to update the canbus name if on a diffrent canbus
    TalonFX m_kickerMotor = new TalonFX(Constants.MotorConstants.kKickerMotorID);

    @Override
    public void setSpeed(double speed) {
        m_kickerMotor.set(speed);
    }

    @Override
    public double getSpeed() {
        return m_kickerMotor.get();
    }

    @Override
    public void updateInputs(KickerIOInputs inputs) {
        inputs.speed = m_kickerMotor.get();
    }
}

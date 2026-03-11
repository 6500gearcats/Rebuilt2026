package frc.robot.subsystems.hopper.kicker;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Constants;

// ! We haven't tested this Spark class. May need to chnage import or smtg
public class SparkKickerIO implements KickerIO {
    private Spark m_kickerMotor;

    public SparkKickerIO() {
        m_kickerMotor = new Spark(Constants.MotorConstants.kKickerMotorID);
    }

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

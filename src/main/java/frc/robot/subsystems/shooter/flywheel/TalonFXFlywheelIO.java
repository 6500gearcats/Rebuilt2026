package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.MotorConstants;

public class TalonFXFlywheelIO implements FlywheelIO {
    TalonFX m_rightMotor;
    TalonFX m_leftMotor;

    VelocityVoltage m_request;
    TalonFXConfiguration talonFXConfigs;

    public TalonFXFlywheelIO() {
        m_leftMotor = new TalonFX(MotorConstants.kShooterMotorLeftID);
        m_rightMotor = new TalonFX(MotorConstants.kShooterMotorRightID);

        m_request = new VelocityVoltage(0).withSlot(0);

        talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.2; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 9; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 5; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.45; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0;

        m_leftMotor.getConfigurator().apply(talonFXConfigs);
        m_rightMotor.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.rightSpeed = m_rightMotor.getVelocity().getValueAsDouble();
        inputs.leftSpeed = m_leftMotor.getVelocity().getValueAsDouble();
    }

    @Override
    public void setSpeed(double speed) {
        // create a velocity closed-loop request, voltage output, slot 0 configs

        // set velocity to rps, add 0.5 V to overcome gravity
        m_rightMotor.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
        m_leftMotor.setControl(new Follower(MotorConstants.kShooterMotorRightID, MotorAlignmentValue.Opposed));
    }

    @Override
    public double getSpeed() {
        return m_leftMotor.getVelocity().getValueAsDouble();
    }
}

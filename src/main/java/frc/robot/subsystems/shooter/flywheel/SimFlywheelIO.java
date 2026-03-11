package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class SimFlywheelIO implements FlywheelIO {
    boolean isSpark = false;

    // Right
    DCMotorSim m_rightMotor;
    TalonFXSimState rightState;

    SparkMax rightMax;
    SparkMaxSim rightMaxSim;

    // Left
    DCMotorSim m_leftMotor;
    TalonFXSimState leftState;

    SparkMax leftMax;
    SparkMaxSim leftMaxSim;

    public SimFlywheelIO() {
    }

    /*
     * Set the speed in either -1 to 1 if using SparkMax
     * otherwise in terms of RadiansPerSec if using TalonFX
     */
    @Override
    public void setSpeed(double speed) {
        System.out.println("Dawg this is unimplemented - SimFlywheelIO");

    }

    public double getSpeed() {
        System.out.println("Dawg this is unimplemented - SimFlywheelIO");
        return 0;
    }

    /*
     * Set the speed in either -1 to 1 if using SparkMax
     * otherwise in terms of RadiansPerSec if using TalonFX
     */
    @Override
    public void updateInputs(FlywheelIOInputs inputs) {
        inputs.leftSpeed = m_leftMotor.getAngularVelocityRadPerSec();
        inputs.rightSpeed = m_rightMotor.getAngularVelocityRadPerSec();
    }
}

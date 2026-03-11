package frc.robot.subsystems.hopper.indexer;

import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.MotorConstants;
import frc.robot.generated.TunerConstants;

public class SimIndexerIO implements IndexerIO {
    boolean isSparkMax = false;

    DCMotorSim m_motor;
    TalonFXSimState simState;

    // create the normal Spark MAX object
    SparkMax max;
    // create the Spark MAX sim object
    SparkMaxSim maxSim;

    public SimIndexerIO(boolean isSparkMax) {
        this.isSparkMax = isSparkMax;
        if (isSparkMax) {
            max = new SparkMax(MotorConstants.kIndexerMotorID, MotorType.kBrushless);
            maxSim = new SparkMaxSim(max, DCMotor.getNeo550(0));
        } else {
            simState = new TalonFXSimState(new CoreTalonFX(MotorConstants.kIndexerMotorID, TunerConstants.kCANBus));
            // Find Constants (kV and kA) using Sys ID
            m_motor = new DCMotorSim(LinearSystemId.createDCMotorSystem(0, 0), DCMotor.getKrakenX60(0));
        }
    }

    /*
     * Sets the speed in either -1 to 1 if using SparkMax
     * otherwise in terms of RadiansPerSec if using TalonFX
     */
    @Override
    public void setSpeed(double speed) {
        if (isSparkMax) {
            maxSim.setVelocity(speed);
        }
        m_motor.setAngularVelocity(speed);
    }

    /*
     * Gets the speed in either -1 to 1 if using SparkMax otherwise in terms of
     * RadiansPerSec if using TalonFX
     */
    @Override
    public double getSpeed() {
        if (isSparkMax) {
            return maxSim.getVelocity();
        }
        return m_motor.getAngularVelocityRadPerSec();
    }

    /*
     * logs the speed in either -1 to 1 if using SparkMaxSim otherwise in terms of
     * RadiansPerSec if using TalonFXSim
     */
    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        if (isSparkMax) {
            inputs.speed = maxSim.getVelocity();
            return;
        }
        inputs.speed = m_motor.getAngularVelocityRadPerSec();
    }

}

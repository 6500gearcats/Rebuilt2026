package frc.robot.subsystems.intake;

import javax.crypto.KEM.Decapsulator;

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

public class SimIntakeIO implements IntakeIO {
    // Deploy
    boolean deployIsSpark = false;
    DCMotorSim m_deployMotor;
    TalonFXSimState deployState;

    SparkMax deployMax;
    SparkMaxSim deployMaxSim;

    // Actual Intake
    boolean intakeIsSpark = false;
    DCMotorSim m_intakeMotor;
    TalonFXSimState intakeState;

    SparkMax intakeMax;
    SparkMaxSim intakeMaxSim;

    public SimIntakeIO(boolean deployIsSpark, boolean intakeIsSpark) {
        this.deployIsSpark = deployIsSpark;
        this.intakeIsSpark = intakeIsSpark;

        if (intakeIsSpark) {
            intakeMax = new SparkMax(MotorConstants.kIntakeMotorID, MotorType.kBrushless);
            intakeMaxSim = new SparkMaxSim(intakeMax, DCMotor.getNeo550(0));
        } else {
            intakeState = new TalonFXSimState(new CoreTalonFX(MotorConstants.kIndexerMotorID, TunerConstants.kCANBus));
            // Find Constants (kV and kA) using Sys ID
            m_intakeMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(0, 0), DCMotor.getKrakenX60(0));
        }

        if (deployIsSpark) {
            deployMax = new SparkMax(MotorConstants.kIntakeDeployMotorID, MotorType.kBrushless);
            deployMaxSim = new SparkMaxSim(intakeMax, DCMotor.getNeo550(0));
        } else {
            deployState = new TalonFXSimState(
                    new CoreTalonFX(MotorConstants.kIntakeDeployMotorID, TunerConstants.kCANBus));
            // Find Constants (kV and kA) using Sys ID
            m_deployMotor = new DCMotorSim(LinearSystemId.createDCMotorSystem(0, 0), DCMotor.getKrakenX60(0));
        }
    }

    /*
     * Set the speed in either -1 to 1 if using SparkMax
     * otherwise in terms of RadiansPerSec if using TalonFX
     */
    @Override
    public void setDeploySpeed(double speed) {
        if (deployIsSpark) {
            deployMaxSim.setVelocity(speed);
            return;
        }
        m_deployMotor.setAngularVelocity(speed);
    }

    /*
     * Sets the speed in either -1 to 1 if using SparkMax
     * otherwise in terms of RadiansPerSec if using TalonFX
     */
    @Override
    public void setIntakeSpeed(double speed) {
        if (intakeIsSpark) {
            intakeMaxSim.setVelocity(speed);
            return;
        }
        m_intakeMotor.setAngularVelocity(speed);
    }

}

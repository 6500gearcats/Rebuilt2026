package frc.robot.subsystems.shooter.turret;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

public class SparkTurretIO implements TurretIO {
    private final SparkMax motor;
    public SparkTurretIO() {
        motor = new SparkMax(Constants.MotorConstants.kTurretYawMotorID, MotorType.kBrushless);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
    }

    @Override
    public void setPosition(double rot) {
        System.out.println("Dawg this is unimplemented - SparkTurretIO");

    }

    @Override
    public void zeroPosition() {
        System.out.println("Dawg this is unimplemented - SparkTurretIO");
    }

    @Override
    public void updateSlotConfigs() {
        System.out.println("Dawg this is unimplemented - SparkTurretIO");
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.motorPositionRotations = 0.0;
        inputs.motorVelocityRps = 0.0;
        inputs.motorAppliedOutput = motor.get();    }

    @Override
    public double getMotorPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public double getSpeed() {
        return motor.getEncoder().getVelocity();
    }
}

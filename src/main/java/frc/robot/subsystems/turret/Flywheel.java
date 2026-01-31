// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;

public class Flywheel extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX m_motor = new TalonFX(Constants.MotorConstants.kShooterMotorRightID, TunerConstants.kCANBus);
  final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);

  TalonFX m_motor2 = new TalonFX(Constants.MotorConstants.kShooterMotorLeftID, TunerConstants.kCANBus);

  public Flywheel() {
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.2; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    m_motor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setTurnSpeed(double speed) {
    // create a velocity closed-loop request, voltage output, slot 0 configs
    // m_motor.set(speed);

    // set velocity to rps, add 0.5 V to overcome gravity
    speed /= 2.0;
    m_motor.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
    m_motor2.setControl(m_request.withVelocity(speed).withFeedForward(0.5));
  }

  public void stopMotor() {
    m_motor.set(0);
    m_motor2.set(0);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;

public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_motor = new TalonFX(50, TunerConstants.kCANBus);
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

  public Turret() {
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    m_motor.getConfigurator().apply(slot0Configs);

    SmartDashboard.putNumber("Turret Position", getPosition());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Turret Position", getPosition());
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  public double getPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public void setPosition(double position) {
    // set position to 10 rotations
    m_motor.setControl(m_request.withPosition(position));
  }
}

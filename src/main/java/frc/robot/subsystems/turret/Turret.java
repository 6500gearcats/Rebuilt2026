// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Turret subsystem that controls the yaw motor and tracks its position.
 */
public class Turret extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(Constants.MotorConstants.kTurretYawMotorID);
  private PositionVoltage m_request;
  private final DigitalInput m_switch = new DigitalInput(4);
  private boolean overridden = false;
  private boolean toZeroPos = false;
  TalonFXConfiguration talonFXConfigs;
  // BOUNDS: 0.0 to 55 rotations

  public Turret() {
    m_request = new PositionVoltage(0).withSlot(2);
    talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.2;
    slot0Configs.kV = 5;
    slot0Configs.kA = 3;
    slot0Configs.kP = 3;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.4;

    var slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = 0.2;
    slot1Configs.kV = SmartDashboard.getNumber("kV", 0);
    slot1Configs.kA = SmartDashboard.getNumber("kA", 0);
    slot1Configs.kP = SmartDashboard.getNumber("kP", 0);
    slot1Configs.kI = 0;
    slot1Configs.kD = SmartDashboard.getNumber("kD", 0);

    // This one is good
    var slot2Configs = talonFXConfigs.Slot2;
    slot2Configs.kS = 0.2;
    slot2Configs.kV = 13;
    slot2Configs.kA = 5;
    slot2Configs.kP = 6;
    slot2Configs.kI = 0;
    slot2Configs.kD = 1;

    m_motor.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("switch on or off", m_switch.get());
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());

    if (toZeroPos) {
      if (!m_switch.get()) {
        m_motor.set(-0.5);
      } else {
        m_motor.set(0);
        zeroMotorPosition();
        toZeroPos = false;
      }
    }
  }

  public void setSpeed(double speed) {
    if (!overridden) {
      if (getMotorPosition() < 2 && speed < 0) {
        speed = 0;
      } else if (getMotorPosition() > 53 && speed > 0) {
        speed = 0;
      }
    }
    m_motor.set(speed);
  }

  public void toggleOverride() {
    overridden = !overridden;
  }

  public double getMotorPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  public void zeroMotorPosition() {
    m_motor.setPosition(0);
  }

  public double getConvertedTurretPosition() {
    return -((getMotorPosition() * 4) - 110);
  }

  public double unconvertPosition(double pos) {
    return ((-1 * pos) + 110) / 4;
  }

  public void setPosition(double deg) {
    double rotations = MathUtil.clamp(unconvertPosition(deg), 2.0, 53.0);
    SmartDashboard.putNumber("UnconvPos", rotations);
    m_motor.setControl(m_request.withPosition(rotations));
  }

  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public void goToZero() {
    toZeroPos = true;
  }

  public void updateSlotConfigs() {
    var slot = talonFXConfigs.Slot1;
    slot.kV = SmartDashboard.getNumber("kV", 0);
    slot.kA = SmartDashboard.getNumber("kA", 0);
    slot.kP = SmartDashboard.getNumber("kP", 0);
    slot.kI = SmartDashboard.getNumber("kI", 0);
    slot.kD = SmartDashboard.getNumber("kD", 0);
    m_request = new PositionVoltage(0).withSlot(1);
  }

  public void setControl(ControlRequest req) {
    m_motor.setControl(req);
  }
}

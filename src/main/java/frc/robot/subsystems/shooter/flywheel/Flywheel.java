// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.flywheel;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.MotorConstants;

/**
 * Flywheel subsystem that controls the shooter motors.
 */
public class Flywheel extends SubsystemBase {
  /** Creates a new Turret. */
  FlywheelIO io;

  FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  public boolean snurboEnable = false;
  public double speedModifier = 1;

  private double speedMultiplier = 1;

  public Flywheel(FlywheelIO io) {
    this.io = io;
  }

  public Flywheel() {
    io = new TalonFXFlywheelIO();
  }

  @Override
  public void periodic() {
    if (snurboEnable) {
      speedModifier = 0.15;
    } else {
      speedModifier = 1;
    }
    SmartDashboard.putNumber("Left Motor Speed", io.getSpeed());
    SmartDashboard.putNumber("Shot Multiplier", speedMultiplier);

    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    if (io == null) {
      System.out.println("Dawg this is null - Flywheel");
      return;
    }
    io.setSpeed(speed);
  }

  public void stopMotor() {
    if (io == null) {
      System.out.println("Dawg this is null - Flywheel");
      return;
    }
    io.setSpeed(0);
  }

  public void incrementMultiplierUp() {
    speedMultiplier++;
  }

  public void incrementMultiplierDown() {
    speedMultiplier--;
  }

  public void updateMotorConfigs() {
  }
}

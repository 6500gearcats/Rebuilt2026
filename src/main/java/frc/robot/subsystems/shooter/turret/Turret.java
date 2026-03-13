// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;

/**
 * Turret subsystem that controls the yaw motor and tracks its position.
 */
public class Turret extends SubsystemBase {
  /** Creates a new Turret. */

  private final DigitalInput m_switch = new DigitalInput(3);
  private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
  // private double tagRot = 0 - tagPose.getRotation().getAngle();
  private boolean overridden = false;
  private boolean toZeroPos = false;
  TalonFXConfiguration talonFXConfigs;
  // BOUNDS: 0.0 to 55

  private TurretIO io;
  TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public Turret(TurretIO io) {
    this.io = io;
  }

  public Turret() {
    io = new TalonFXTurretIO();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
    SmartDashboard.putNumber("Robot Rot in Deg", robotStateMachine.getPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Turret Zeroing", toZeroPos);
    SmartDashboard.putBoolean("Turret Limit Switch", m_switch.get());

    if (toZeroPos) {
      if (!m_switch.get()) {
        io.setSpeed(-0.5);
      } else {
        io.setSpeed(0);
        zeroMotorPosition();
        toZeroPos = false;
      }
    }
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setSpeed(double speed) {
    if (io == null) {
      System.out.println("Dawg this is null - Turret");
      return;
    }

    if (!overridden) {
      if ((getMotorPosition() < 2)) {
        if (speed < 0) {
          speed = 0;
        }
      } else if ((getMotorPosition() > 53)) {
        if (speed > 0) {
          speed = 0;
        }
      }
    }
    io.setSpeed(speed);
  }

  public void toggleOverride() {
    if (io == null) {
      System.out.println("Dawg this is null - Turret");
      return;
    }
    overridden = !overridden;
  }

  /**
   * Returns the raw motor position in rotations.
   *
   * @return motor sensor position
   */
  public double getMotorPosition() {
    if (io == null) {
      System.out.println("Dawg this is null - Turret");
      return 0;
    }
    return io.getMotorPosition();
  }

  public void zeroMotorPosition() {
    if (io == null) {
      System.out.println("Dawg this is null - Turret");
      return;
    }
    io.setPosition(0);
  }

  /**
   * Returns the turret position converted to degrees.
   * Will only work with current gearbox. Update values based on Robot
   *
   * @return turret angle in degrees
   */
  public double getConvertedTurretPosition() {
    return -((getMotorPosition() * 4) - 110);
  }

  public double unconvertPosition(double pos) {
    return ((-1 * pos) + 110) / 4;
  }

  /**
   * Moves the turret to the given position setpoint.
   *
   * @param deg desired position in degress
   */
  public void setPosition(double deg) {
    if (io == null) {
      System.out.println("Dawg this is null - Turret");
      return;
    }
    io.setPosition(deg);
  }

  public void goToZero() {
    toZeroPos = true;
  }

  public void updateSlotConfigs() {

  }
}

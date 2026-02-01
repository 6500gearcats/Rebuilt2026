// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotStateMachine;
import frc.robot.generated.TunerConstants;

/**
 * Turret subsystem that controls the yaw motor and tracks its position.
 */
public class Turret extends SubsystemBase {
  /** Creates a new Turret. */
  private final TalonFX m_motor = new TalonFX(Constants.MotorConstants.kTurretYawMotorID);
  private final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
  private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get();
  private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
  // private double tagRot = 0 - tagPose.getRotation().getAngle();

  public Turret() {
    // in init function, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0; // An error of 1 rotation results in 2.4 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity of 1 rps results in 0.1 V output

    m_motor.getConfigurator().apply(slot0Configs);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
  }

  public void setSpeed(double speed) {
    m_motor.set(speed);
  }

  /**
   * Returns the raw motor position in rotations.
   *
   * @return motor sensor position
   */
  public double getMotorPosition() {
    return m_motor.getPosition().getValueAsDouble();
  }

  /**
   * Returns the turret position converted to degrees.
   *
   * @return turret angle in degrees
   */
  public double getConvertedTurretPosition() {
    return getMotorPosition() / 90;
  }

  /**
   * Moves the turret to the given position setpoint.
   *
   * @param position desired position in motor rotations
   */
  public void setPosition(double position) {
    // set position to 10 rotations
    m_motor.setControl(m_request.withPosition(position));
  }
}

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

  // BOUNDS: 0.0 to 55

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
    SmartDashboard.putNumber("Robot Rot in Deg", robotStateMachine.getPose().getRotation().getDegrees());

    SmartDashboard.putNumber("AlignRate", getAlignRate());
  }

  public void setSpeed(double speed) {
    if((getMotorPosition() < 2)) {
      if(speed < 0) {
        speed = 0;
      }
    }
    else if((getMotorPosition() > 53)) {
      if(speed > 0) {
        speed = 0;
      }
    }
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

  public void zeroMotorPosition() {
    m_motor.setPosition(0);
  }

  /**
   * Returns the turret position converted to degrees.
   *
   * @return turret angle in degrees
   */
  public double getConvertedTurretPosition() {
    return -((getMotorPosition() * 4) - 110);

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
  public double getAlignRate() {
                // This method will be called once per scheduler run
                double rectW = (tagPose.getX() - robotStateMachine.getPose().getX());
                double rectH = (tagPose.getY() - 1 - robotStateMachine.getPose().getY());

                SmartDashboard.putNumber("rectH", rectH);
                SmartDashboard.putNumber("rectW", rectW);
                double poseRot = robotStateMachine.getPose().getRotation().getDegrees();// + getConvertedTurretPosition();
                SmartDashboard.putNumber("Turret on field", poseRot);

                double newAngle = Math.toDegrees( Math.atan2(rectH, rectW)); // gets wanted angle for robot field
                                                                              // oriented
                
                //SmartDashboard.putNumber("newAngle", newAngle);
                double newAngleRate;
                
                newAngleRate = ((newAngle - poseRot));
                // if (poseRot > 0) {
                //         newAngleRate = ((newAngle - poseRot));
                // } else {
                //         newAngleRate = ((poseRot - newAngle));
                // }

                double kP = 0.003;

                double newNewAngleRate = newAngleRate * kP;
                // setPosition(newAngle);
                //SmartDashboard.putNumber("newAngleRate", newAngleRate);
                return newNewAngleRate;
              }
}

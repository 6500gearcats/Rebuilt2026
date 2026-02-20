// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
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
  private final PositionVoltage m_request;
  private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(20).get();
  private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();
  // private double tagRot = 0 - tagPose.getRotation().getAngle();
  private boolean overridden = false;
  // BOUNDS: 0.0 to 55

  public Turret() {
    m_request = new PositionVoltage(0).withSlot(0);
    var talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.2; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 5; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 3; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 3; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.5; // A velocity error of 1 rps results in 0.1 V output

    m_motor.getConfigurator().apply(talonFXConfigs);

    m_motor.getConfigurator();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Motor Position", getMotorPosition());
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
    SmartDashboard.putNumber("Robot Rot in Deg", robotStateMachine.getPose().getRotation().getDegrees());
  }

  public void setSpeed(double speed) {
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
    m_motor.set(speed);
  }

  public void toggleOverride() {
    overridden = !overridden;
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

  public double unconvertPosition(double pos) {
    return ((-1 * pos) + 110) / 4;
  }

  /**
   * Moves the turret to the given position setpoint.
   *
   * @param deg desired position in degress
   */
  public void setPosition(double deg) {
    SmartDashboard.putNumber("UnconvPos", unconvertPosition(deg));
    m_motor.setControl(m_request.withPosition(unconvertPosition(deg)));
  }
}

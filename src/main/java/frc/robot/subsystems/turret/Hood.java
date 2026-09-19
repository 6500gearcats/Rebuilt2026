// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/** Controls the turret hood and reports its absolute angle sensor. */
public class Hood extends SubsystemBase {
  private final TalonFX m_motor = new TalonFX(MotorConstants.kTurretHoodID);
  private final CANcoder m_encoder = new CANcoder(MotorConstants.kTurretHoodEncoderID);
  public Hood(){
    // Configure the CANcoder for basic use

//    final CANcoderConfiguration kEncoder1Config = new CANcoderConfiguration()
//            .withMagnetSensor(new MagnetSensorConfigs()
//                    .withAbsoluteSensorDiscontinuityPoint(1.0)
//                    .withMagnetOffset(kEncoder1Offset));
//
//    CANcoderConfiguration configs = new CANcoderConfiguration();
//    // This CANcoder should report absolute position from [-0.5, 0.5) rotations,
//    // with a 0.26 rotation offset, with clockwise being positive
////    configs.MagnetSensor.. = AbsoluteSensorRange.Signed_PlusMinusHalf;
//    configs.MagnetSensor.MagnetOffset = 0.26;
//    configs.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    // Write these configs to the CANcoder
//    m_encoder.getConfigurator().apply(configs);

    // Set the position to 0 rotations for initial use
//    m_encoder.setPosition(0);


  }
  @Override
  public void periodic() {
    double absolutePositionRotations = getAbsolutePositionRotations();
    SmartDashboard.putNumber("Hood Absolute Position (rotations)", absolutePositionRotations);
    SmartDashboard.putNumber("Hood Absolute Position (degrees)", absolutePositionRotations * 360.0);
  }

  /** Runs the hood motor at the requested duty cycle. */
  public void setSpeed(double speed) {
    m_motor.set(MathUtil.clamp(speed, -1.0, 1.0));
  }

  /** Stops the hood motor. */
  public void stop() {
    m_motor.stopMotor();
  }

  /** Returns the raw absolute encoder position in rotations. */
  public double getAbsolutePositionRotations() {
    return m_encoder.getAbsolutePosition().getValueAsDouble();
  }

  /** Returns the raw absolute encoder position in degrees. */
  public double getAbsolutePositionDegrees() {
    return getAbsolutePositionRotations() * 360.0;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.DoubleSupplier;
import java.util.logging.Logger;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.utility.RangeFinder;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotStateMachine.FieldZone;

/**
 * Flywheel subsystem that controls the shooter motors.
 */
public class Flywheel extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX m_motor = new TalonFX(Constants.MotorConstants.kShooterMotorRightID);
  VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  public boolean snurboEnable = false;
  public double speedModifier = 1;
  public boolean waitForSpeed = false;
  private double speedMultiplier = 0;
  public double rotationMultiplier = 0;
  private double reqSpeed;
  private double speedWithinToleranceSince = -1;
  private boolean speedStable = false;
  private static final double SPEED_TOLERANCE_RPS = 2.0;
  private static final double SPEED_STABLE_TIME_SECONDS = 0.08;
  TalonFX m_motor2 = new TalonFX(Constants.MotorConstants.kShooterMotorLeftID);
  private RobotStateMachine robotStateMachine;

  TalonFXConfiguration talonFXConfigs;

  // TODO: Add a constant Spin to the motors to not have to fight static friction

  public Flywheel(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    talonFXConfigs = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.6));

    // // set slot 0 gains
    // var slot0Configs = talonFXConfigs.Slot0;
    // slot0Configs.kS = 0.3087; // Add 0.25 V output to overcome static friction
    // slot0Configs.kV = 0.076456; // A velocity target of 1 rps results in 0.12 V
    // output
    // slot0Configs.kA = 0.010904; // An acceleration of 1 rps/s requires 0.01 V
    // output
    // slot0Configs.kP = 0.026467; // A position error of 2.5 rotations results in
    // 12 V output
    // slot0Configs.kI = 0; // no output for integrated error
    // slot0Configs.kD = 0;

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.3087; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.076456; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.010904; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.3; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.00005;

    m_motor.getConfigurator().apply(talonFXConfigs);
    m_motor2.getConfigurator().apply(talonFXConfigs);
    m_motor2.setControl(new Follower(MotorConstants.kShooterMotorRightID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void periodic() {
    RobotStateMachine.ShotSolution shotSolution = robotStateMachine.getShotSolution();

    if (snurboEnable) {
      speedModifier = 0.15;// 0.15;
    } else {
      speedModifier = 1;
    }
    if (!robotStateMachine.isFacingHub()) {
      rotationMultiplier = 2;
    } else {
      rotationMultiplier = 0;
    }

    if (robotStateMachine.isActive() /*
                                      * && robotStateMachine.checkZone() ==
                                      * FieldZone.ALLIANCE
                                      */) {
      if (shotSolution.getDistance() > 0.0) {
        setSpeed(shotSolution.getFlywheelSpeed());
      }
    }

    updateSpeedReadiness();

    SmartDashboard.putNumber("Left Motor Speed", m_motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shot Multiplier", speedMultiplier);
    SmartDashboard.putNumber("Rotation Multiplier", rotationMultiplier);

    SmartDashboard.putBoolean("Up to Speed", isUpToSpeed());
    SmartDashboard.putNumber("reqSpeed", reqSpeed);
    SmartDashboard.putNumber("actSpeed", getSpeed());
    SmartDashboard.putBoolean("isUnderTrench", robotStateMachine.underTrench());
    SmartDashboard.putNumber("rot new testing", robotStateMachine.getConvertedTurretPosition());
    SmartDashboard.putNumber("rot adder",
        RangeFinder.getRotAdder(robotStateMachine.getConvertedTurretPosition()));
    SmartDashboard.putNumber("rot old testing", robotStateMachine.getTurretPose().getRotation().getDegrees());

    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    double trenchCorr = 0;
    if (robotStateMachine.ductTapeCorrection) {
      trenchCorr = 4;
    }
    // set velocity to rps, add 0.5 V to overcome gravity
    SmartDashboard.putNumber("flywheel initial speed", speed);
    double speedValue = speed + (2 * speedMultiplier)
        + RangeFinder.getRotAdder(robotStateMachine.getConvertedTurretPosition());
    if (speedValue > 0) {
      SmartDashboard.putNumber("flywheel sped-up speed", speedValue);

      if (robotStateMachine.underTrench()) {
        speedValue = 68 + (2 * speedMultiplier) + rotationMultiplier + trenchCorr;
      }
      reqSpeed = speedValue;
      m_motor.setControl(m_request.withVelocity(speedValue));
    }
  }

  /*
   * Gets Speed in RPS
   */
  public double getSpeed() {
    return m_motor.getVelocity().getValueAsDouble();
  }

  public double getReqSpeed() {
    return reqSpeed;
  }

  public boolean isUpToSpeed() {
    return speedStable;
  }

  private void updateSpeedReadiness() {
    double speedError = Math.abs(reqSpeed - getSpeed());
    if (speedError <= SPEED_TOLERANCE_RPS) {
      if (speedWithinToleranceSince < 0) {
        speedWithinToleranceSince = Timer.getFPGATimestamp();
      }
      speedStable = Timer.getFPGATimestamp() - speedWithinToleranceSince >= SPEED_STABLE_TIME_SECONDS;
    } else {
      speedWithinToleranceSince = -1;
      speedStable = false;
    }
  }

  public void stopMotor() {
    m_motor.set(0);
    m_motor2.set(0);
  }

  public void incrementMultiplierUp() {
    speedMultiplier++;
  }

  public void incrementMultiplierDown() {
    speedMultiplier--;
  }

  public void setControl(ControlRequest req) {
    m_motor.setControl(req);
    // m_motor2.setControl(req);
  }

  public void updateMotorConfigs() {
    // var slot = talonFXConfigs.Slot0;
    // slot.kV = SmartDashboard.getNumber("shooter kV", 0);
    // slot.kA = SmartDashboard.getNumber("shooter kA", 0);
    // slot.kP = SmartDashboard.getNumber("shooter kP", 0);
    // slot.kI = SmartDashboard.getNumber("shooter kI", 0);
    // slot.kD = SmartDashboard.getNumber("shooter kD", 0);
    // m_motor.getConfigurator().apply(slot);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.OptionalDouble;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.utility.RangeFinder;
import frc.robot.Constants.MotorConstants;

/**
 * Flywheel subsystem that controls the shooter motors.
 */
public class Flywheel extends SubsystemBase {
  /** Creates a new Turret. */
  TalonFX m_topMotor = new TalonFX(Constants.MotorConstants.kShooterMotorTopID);
  VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  public boolean snurboEnable = false;
  public double speedModifier = 1;
  public boolean waitForSpeed = false;
  private double speedMultiplier = 0;
  public double rotationMultiplier = 0;
  private double reqSpeed;
  private double speedWithinToleranceSince = -1;
  private OptionalDouble manualSpeedOverride = OptionalDouble.empty();
  private int m_loop = 0;

  private boolean speedStable = false;
  private static final double SPEED_TOLERANCE_RPS = 2.0;
  private static final double SPEED_STABLE_TIME_SECONDS = 0.08;

  TalonFX m_bottomMotor = new TalonFX(Constants.MotorConstants.kShooterMotorBottomID);
  private RobotStateMachine robotStateMachine;

  TalonFXConfiguration talonFXConfigs;

  // TODO: Add a constant Spin to the motors to not have to fight static friction

  public Flywheel(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    talonFXConfigs = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.5))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

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
    slot0Configs.kS = 0.28342; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.075434; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.0055825; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 0.3; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.00005;

    m_topMotor.getConfigurator().apply(talonFXConfigs);
    m_bottomMotor.getConfigurator().apply(talonFXConfigs);
    m_bottomMotor.setControl(new Follower(MotorConstants.kShooterMotorTopID, MotorAlignmentValue.Aligned));
  }

  @Override
  public void periodic() {
    RobotStateMachine.ShotSolution shotSolution = robotStateMachine.getShotSolution();

    if (snurboEnable) {
      speedModifier = 0.15;
    } else {
      speedModifier = 1;
    }
    if (!robotStateMachine.isFacingHub()) {
      rotationMultiplier = 2;
    } else {
      rotationMultiplier = 0;
    }

    if (manualSpeedOverride.isPresent()) {
      setSpeed(manualSpeedOverride.getAsDouble());
    } else if (robotStateMachine.isActive() /*
                                      * && robotStateMachine.checkZone() ==
                                      * FieldZone.ALLIANCE
                                      */) {

      if (shotSolution.getDistance() > 0.0) {
        setSpeed(shotSolution.getFlywheelSpeed());
      }
    }
    updateSpeedReadiness();

    // if (m_loop == 20){
      // m_loop = 0;
      SmartDashboard.putNumber("Left Motor Speed", m_topMotor.getVelocity().getValueAsDouble());
      SmartDashboard.putNumber("Shot Multiplier", speedMultiplier);
      SmartDashboard.putNumber("Rotation Multiplier", rotationMultiplier);

      SmartDashboard.putBoolean("Up to Speed", isUpToSpeed());
      SmartDashboard.putNumber("reqSpeed", reqSpeed);
      SmartDashboard.putBoolean("Flywheel Manual Override", manualSpeedOverride.isPresent());
      SmartDashboard.putNumber("actSpeed", getSpeed());
      SmartDashboard.putBoolean("isUnderTrench", robotStateMachine.underTrench());
      SmartDashboard.putNumber("rot new testing", robotStateMachine.getConvertedTurretPosition());
      SmartDashboard.putNumber("rot adder",
          RangeFinder.getRotAdder(robotStateMachine.getConvertedTurretPosition()));
      SmartDashboard.putNumber("rot old testing", robotStateMachine.getTurretPose().getRotation().getDegrees());
    // }
    // m_loop++;
    // This method will be called once per scheduler run
  }

  /** Selects an exact speed in RPS until manual mode is cleared. */
  public void setManualSpeed(double speed) {
    manualSpeedOverride = OptionalDouble.of(speed);
    setSpeed(speed);
  }

  public void clearManualSpeed() {
    manualSpeedOverride = OptionalDouble.empty();
  }

  public void setSpeed(double speed) {
    // Stop requests still take effect even when a manual preset is selected.
    double speedValue = 0;
    if (speed > 0) {
      if (manualSpeedOverride.isPresent()) {
        speed = manualSpeedOverride.getAsDouble();
        speedValue = speed;
      } else {
        speedValue = speed + (2 * speedMultiplier)
            + RangeFinder.getRotAdder(robotStateMachine.getConvertedTurretPosition());
        if (robotStateMachine.underTrench()) {
          double trenchCorr = robotStateMachine.ductTapeCorrection ? 4 : 0;
          speedValue = 68 + (2 * speedMultiplier) + rotationMultiplier + trenchCorr;
        }
      }
    }
    reqSpeed = speedValue;
    SmartDashboard.putNumber("flywheel initial speed", speed);
    SmartDashboard.putNumber("flywheel new speed", speedValue);
    m_topMotor.setControl(m_request.withVelocity(speedValue));
  }

  /*
   * Gets Speed in RPS
   */
  public double getSpeed() {
    return m_topMotor.getVelocity().getValueAsDouble();
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
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  public void incrementMultiplierUp() {
    speedMultiplier++;
  }

  public void incrementMultiplierDown() {
    speedMultiplier--;
  }

  public void setControl(ControlRequest req) {
    m_topMotor.setControl(req);
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

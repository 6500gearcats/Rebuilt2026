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
import frc.robot.DashboardTelemetry;
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
  private double reqSpeed;
  private double baseSpeed;
  private double rotationCorrection;
  private double speedWithinToleranceSince = -1;
  private OptionalDouble manualSpeedOverride = OptionalDouble.empty();
  private final DashboardTelemetry.DebugPublisher debugTelemetry = new DashboardTelemetry.DebugPublisher();

  private boolean speedStable = false;
  private boolean measuredSpeedStable = false;
  private static final double SPEED_TOLERANCE_RPS = 2.0;
  private static final double SPEED_STABLE_TIME_SECONDS = 0.08;
  // Preserve the existing test override until shooting behavior is reviewed separately.
  private static final boolean SPEED_READINESS_OVERRIDE = true;

  TalonFX m_bottomMotor = new TalonFX(Constants.MotorConstants.kShooterMotorBottomID);
  private RobotStateMachine robotStateMachine;

  TalonFXConfiguration talonFXConfigs;

  // TODO: Add a constant Spin to the motors to not have to fight static friction

  public Flywheel(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    talonFXConfigs = new TalonFXConfiguration().withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.6))
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

    publishTelemetry();
  }

  private void publishTelemetry() {
    // Retain the existing operating keys for dashboard compatibility.
    SmartDashboard.putBoolean("Up to Speed", isUpToSpeed());
    SmartDashboard.putBoolean("Flywheel Speed Within Tolerance", measuredSpeedStable);
    SmartDashboard.putBoolean("Flywheel Readiness Override", SPEED_READINESS_OVERRIDE);
    SmartDashboard.putNumber("reqSpeed", reqSpeed);
    SmartDashboard.putNumber("actSpeed", getSpeed());
    SmartDashboard.putBoolean("Flywheel Manual Override", manualSpeedOverride.isPresent());
    SmartDashboard.putBoolean("isUnderTrench", robotStateMachine.underTrench());
    if (debugTelemetry.shouldPublish()) {
      SmartDashboard.putNumber("Debug/Flywheel/Base Speed (RPS)", baseSpeed);
      SmartDashboard.putNumber("Debug/Flywheel/Rotation Correction (RPS)", rotationCorrection);
      SmartDashboard.putNumber("Debug/Flywheel/Requested Speed (RPS)", reqSpeed);
    }
  }

  /** Selects an exact speed in RPS until manual mode is cleared. */
  public void setManualSpeed(double speed) {
    manualSpeedOverride = OptionalDouble.of(speed);
    setSpeed(speed);
  }

  public void setManualSpeedIncrement(double increment) {
    double newSpeed = manualSpeedOverride.orElse(getReqSpeed()) + increment;
    manualSpeedOverride = OptionalDouble.of(newSpeed);
    setSpeed(newSpeed);
  }

  public void clearManualSpeed() {
    manualSpeedOverride = OptionalDouble.empty();
  }

  public void setSpeed(double speed) {
    // Stop requests still take effect even when a manual preset is selected.
    double speedValue = 0;
    rotationCorrection = 0;
    if (speed > 0) {
      if (manualSpeedOverride.isPresent()) {
        speed = manualSpeedOverride.getAsDouble();
        speedValue = speed;
      } else {
        rotationCorrection = RangeFinder.getRotAdder(robotStateMachine.getConvertedTurretPosition());
        speedValue = speed + rotationCorrection;
      }
    }
    reqSpeed = speedValue;
    baseSpeed = speed > 0 ? speed : 0;
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
      measuredSpeedStable = Timer.getFPGATimestamp() - speedWithinToleranceSince >= SPEED_STABLE_TIME_SECONDS;
    } else {
      speedWithinToleranceSince = -1;
      measuredSpeedStable = false;
    }
    speedStable = SPEED_READINESS_OVERRIDE || measuredSpeedStable;
  }

  public void stopMotor() {
    m_topMotor.set(0);
    m_bottomMotor.set(0);
  }

  public void setControl(ControlRequest req) {
    m_topMotor.setControl(req);
    // m_motor2.setControl(req);
  }
}

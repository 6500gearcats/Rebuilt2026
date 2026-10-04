// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; see the WPILib BSD license file in this project.
package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;
import frc.robot.RobotStateMachine;
import frc.robot.utility.shooting.SetpointReadiness;

/** Controls one velocity leader and one follower; setSpeed takes exact mechanism RPS. */
public class Flywheel extends SubsystemBase {
  private final TalonFX m_topMotor = new TalonFX(MotorConstants.kShooterMotorTopID);
  private final TalonFX m_bottomMotor = new TalonFX(MotorConstants.kShooterMotorBottomID);
  private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0);
  private final SetpointReadiness readiness = new SetpointReadiness(2.0, 0.08);
  private final RobotStateMachine robotStateMachine;
  private final boolean configured;
  private double reqSpeed;
  private int m_loop;
  public boolean snurboEnable = false;
  public double speedModifier = 1;

  public Flywheel(RobotStateMachine robotStateMachine) {
    this.robotStateMachine = robotStateMachine;
    TalonFXConfiguration configs = new TalonFXConfiguration()
        .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(0.6))
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    // Retain the merged gains/inversion until physical direction and speed tests.
    configs.Slot0.kS = 0.28342;
    configs.Slot0.kV = 0.075434;
    configs.Slot0.kA = 0.0055825;
    configs.Slot0.kP = 0.3;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0.00005;
    boolean topConfigured = m_topMotor.getConfigurator().apply(configs).isOK();
    boolean bottomConfigured = m_bottomMotor.getConfigurator().apply(configs).isOK();
    boolean followerConfigured = m_bottomMotor.setControl(
        new Follower(m_topMotor.getDeviceID(), MotorAlignmentValue.Aligned)).isOK();
    configured = topConfigured && bottomConfigured && followerConfigured;
    if (!configured) {
      DriverStation.reportError("Flywheel configuration failed; shooting disabled", false);
    }
  }

  @Override
  public void periodic() {
    speedModifier = snurboEnable ? 0.15 : 1;
    var velocity = m_topMotor.getVelocity();
    var followerVelocity = m_bottomMotor.getVelocity();
    boolean healthy = configured && DriverStation.isEnabled()
        && velocity.getStatus().isOK() && followerVelocity.getStatus().isOK();
    boolean followerAtSpeed = Math.abs(followerVelocity.getValueAsDouble() - reqSpeed) <= 2.0;
    readiness.update(reqSpeed, velocity.getValueAsDouble(), healthy && reqSpeed > 0 && followerAtSpeed,
        Timer.getFPGATimestamp());
    if (!healthy) { stopMotor(); }
    // Retain the existing slower auxiliary telemetry cadence.
    if (++m_loop >= 20) {
      m_loop = 0;
      SmartDashboard.putNumber("Top Motor Speed", velocity.getValueAsDouble());
      SmartDashboard.putNumber("Bottom Motor Speed", followerVelocity.getValueAsDouble());
    }
    SmartDashboard.putBoolean("Up to Speed", isUpToSpeed());
    SmartDashboard.putNumber("reqSpeed", reqSpeed);
    SmartDashboard.putNumber("actSpeed", getSpeed());
  }

  /** Commands exact mechanism rotations/second; no hidden range or angle corrections. */
  public void setSpeed(double speedRps) {
    if (!configured || !Double.isFinite(speedRps) || speedRps <= 0 || !DriverStation.isEnabled()) {
      stopMotor();
      return;
    }
    if (Math.abs(speedRps - reqSpeed) > 2.0) { readiness.reset(); }
    reqSpeed = speedRps;
    if (!m_topMotor.setControl(m_request.withVelocity(speedRps)).isOK()) { stopMotor(); }
  }

  /** Leader mechanism RPS, after the configured sensor-to-mechanism ratio. */
  public double getSpeed() { return m_topMotor.getVelocity().getValueAsDouble(); }
  public double getReqSpeed() { return reqSpeed; }
  public boolean isUpToSpeed() {
    var top = m_topMotor.getVelocity();
    var bottom = m_bottomMotor.getVelocity();
    return reqSpeed > 0 && readiness.isReady() && top.getStatus().isOK() && bottom.getStatus().isOK()
        && Math.abs(reqSpeed - top.getValueAsDouble()) <= 2.0
        && Math.abs(reqSpeed - bottom.getValueAsDouble()) <= 2.0;
  }

  /** Stop only the leader: the follower keeps following through the next restart. */
  public void stopMotor() {
    reqSpeed = 0;
    readiness.reset();
    m_topMotor.stopMotor();
  }

  /** Existing POV controls tune manual RPS or apply an explicit automatic trim. */
  public void incrementMultiplierUp() { robotStateMachine.getShotCalibration().adjustFlywheel(1); }
  public void incrementMultiplierDown() { robotStateMachine.getShotCalibration().adjustFlywheel(-1); }

  /** Direct characterization request; cannot authorize feeding. */
  public void setControl(ControlRequest request) {
    reqSpeed = 0;
    readiness.reset();
    m_topMotor.setControl(request);
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorConstants;

/**
 * Controls the ground intake mechanism, which consists of two independent motors:
 * <ul>
 *   <li><b>Roller motor</b> ({@code m_intakeMotor}) — spins intake wheels to pull notes
 *       from the floor into the hopper. Open-loop, set by {@link #setIntakeSpeed(double)}.
 *   <li><b>Deploy motor</b> ({@code m_intakeDeployMotor}) — rotates the intake arm up or
 *       down. Open-loop, set by {@link #deployIntake(double)}.
 * </ul>
 *
 * <p>Both motors operate in open-loop (duty-cycle percent output). There is no position
 * or velocity closed-loop on this subsystem — the driver holds the deploy at a fixed speed
 * while the intake is active.
 */
public class Intake extends SubsystemBase {
  private final TalonFX m_intakeMotor = new TalonFX(MotorConstants.kIntakeMotorID);
  private final TalonFX m_intakeDeployMotor = new TalonFX(MotorConstants.kIntakeDeployMotorID);

  public Intake() {}

  /**
   * Publishes telemetry and seeds CTRE simulation state each loop.
   *
   * <p>In simulation, the CTRE Phoenix 6 sim state is seeded with velocity and position values
   * derived from the commanded duty cycle and a nominal free-speed assumption. This makes
   * SmartDashboard signals display realistic values even without physical motors.
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake/DeployPositionRot",   m_intakeDeployMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerVelocityRPS",   m_intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake/DeployStatorCurrentA", m_intakeDeployMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Intake/RollerStatorCurrentA", m_intakeMotor.getStatorCurrent().getValueAsDouble());

    if (RobotBase.isSimulation()) {
      // Seed CTRE sim state so velocity/position signals read realistic values in sim telemetry.
      final double kFreeSpeedRPS = 100.0;
      final double kLoopPeriodS = 0.02;
      m_intakeMotor.getSimState().setRotorVelocity(m_intakeMotor.get() * kFreeSpeedRPS);
      double deployVelRPS = m_intakeDeployMotor.get() * kFreeSpeedRPS;
      m_intakeDeployMotor.getSimState().setRotorVelocity(deployVelRPS);
      m_intakeDeployMotor.getSimState().addRotorPosition(deployVelRPS * kLoopPeriodS);
    }
  }

  /**
   * Sets the roller motor duty cycle.
   *
   * @param speed Duty cycle in [−1, 1]. Positive pulls notes inward toward the hopper.
   */
  public void setIntakeSpeed(double speed) {
    m_intakeMotor.set(speed);
  }

  /**
   * Sets the deploy arm motor duty cycle to hold the arm at the commanded position.
   *
   * <p>Note: this does not extend or retract the arm to a specific angle — it applies
   * a constant duty-cycle output to resist gravity and hold the arm deployed while
   * the intake is active. Typically called with a small positive value (e.g., 0.15)
   * from {@link frc.robot.commands.RunIntake}.
   *
   * @param speed Duty cycle in [−1, 1]. Positive rotates toward the deployed (down) position.
   */
  public void deployIntake(double speed){
    m_intakeDeployMotor.set(speed);
  }
}

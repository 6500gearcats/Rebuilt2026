// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Hopper subsystem — two motors that move balls from the storage area into the shooter.
 *
 * <h2>Mechanism Overview</h2>
 * The hopper has two stages:
 * <ul>
 *   <li><b>Indexer ({@code m_hopperMotor})</b>: belt or wheel mechanism that advances balls through
 *       the storage area toward the kicker. Typically runs at lower speed to pace ball delivery.
 *   <li><b>Kicker ({@code m_kickerMotor})</b>: final accelerator stage that launches the ball
 *       into the shooter flywheel at the right entry speed. Runs at full speed (positive duty cycle).
 * </ul>
 * The two speeds can differ — the negative indexer speed ({@code −0.9}) and positive kicker speed
 * ({@code +1}) in {@link frc.robot.commands.ShootWhenReady ShootWhenReady} reflect a reversal in
 * the mechanism geometry: the indexer belt runs "backward" (negative) to push balls forward.
 *
 * <h2>Integration with ShootWhenReady</h2>
 * {@link frc.robot.commands.ShootWhenReady ShootWhenReady} calls {@link #startAllMotors} from
 * inside a {@code startEnd} command. The {@code startEnd} pattern guarantees that {@link #stopAllMotors}
 * is called when the command ends or is interrupted — critical in autonomous to prevent the hopper
 * from running into the next path segment after a shot timeout expires.
 *
 * <h2>Note on IO Layer</h2>
 * Unlike the turret and shooter, the hopper does not use an IO interface. It drives the TalonFX
 * motors directly with percent-output duty-cycle control ({@code .set(speed)}), which is sufficient
 * for this open-loop application — no position or velocity feedback is needed for ball feeding.
 */
public class Hopper extends SubsystemBase {
  TalonFX m_hopperMotor = new TalonFX(Constants.MotorConstants.kIndexerMotorID);
  TalonFX m_kickerMotor = new TalonFX(Constants.MotorConstants.kKickerMotorID);

  public Hopper() {
  }

  @Override
  public void periodic() {
    // Monitor motor state for jam detection and post-match analysis.
    // A kicker velocity near zero while hopperSpeed is high suggests a ball jam.
    SmartDashboard.putNumber("Hopper/IndexerVelocityRPS",    m_hopperMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/IndexerStatorCurrentA", m_hopperMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/KickerVelocityRPS",     m_kickerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/KickerStatorCurrentA",  m_kickerMotor.getStatorCurrent().getValueAsDouble());
  }

  /**
   * Starts both motors at the specified duty-cycle speeds.
   *
   * <p>Typically called from {@code startEnd} inside {@link frc.robot.commands.ShootWhenReady}
   * with {@code hopperSpeed = −0.9} (indexer belt reversed into shooter) and
   * {@code kickerSpeed = +1.0} (kicker at full speed).
   *
   * @param hopperSpeed indexer duty cycle (−1.0 to +1.0); negative = toward shooter in this geometry
   * @param kickerSpeed kicker duty cycle (−1.0 to +1.0); positive = launching into flywheel
   */
  public void startAllMotors(double hopperSpeed, double kickerSpeed) {
    m_hopperMotor.set(hopperSpeed);
    m_kickerMotor.set(kickerSpeed);
  }

  /**
   * Stops both motors immediately.
   *
   * <p>Called by the {@code startEnd} end block in {@link frc.robot.commands.ShootWhenReady}
   * whenever that command ends — whether normally, via timeout, or via interrupt. This prevents
   * the hopper from running uncontrolled between auto path segments.
   */
  public void stopAllMotors() {
    m_hopperMotor.set(0);
    m_kickerMotor.set(0);
  }

  /** Sets the indexer motor speed independently. Used for fine-tuned feeding sequences. */
  public void setHopperSpeed(double speed) {
    m_hopperMotor.set(speed);
  }

  /** Sets the kicker motor speed independently. */
  public void setKickerMotorSpeed(double speed) {
    m_kickerMotor.set(speed);
  }
}

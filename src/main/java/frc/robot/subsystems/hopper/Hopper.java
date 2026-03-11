// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.indexer.IndexerIO;
import frc.robot.subsystems.hopper.indexer.IndexerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.indexer.TalonFXIndexerIO;
import frc.robot.subsystems.hopper.kicker.KickerIO;
import frc.robot.subsystems.hopper.kicker.KickerIOInputsAutoLogged;
import frc.robot.subsystems.hopper.kicker.TalonFXKickerIO;

/**
 * Hopper subsystem that controls the indexer and kicker motors.
 */
public class Hopper extends SubsystemBase {
  /** Creates a new Hopper. */

  IndexerIO m_indexerIO;
  KickerIO m_kickerIO;

  IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();
  KickerIOInputsAutoLogged kickerInputs = new KickerIOInputsAutoLogged();

  public Hopper(IndexerIO indexerIO, KickerIO kickerIO) {
    m_indexerIO = indexerIO;
    m_kickerIO = kickerIO;
  }

  public Hopper() {
    m_indexerIO = new TalonFXIndexerIO();
    m_kickerIO = new TalonFXKickerIO();
  }

  @Override
  public void periodic() {
    m_indexerIO.updateInputs(indexerInputs);
    m_kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer", indexerInputs);
    Logger.processInputs("Kicker", kickerInputs);
  }

  /**
   * Starts both hopper and kicker motors.
   *
   * @param hopperSpeed speed for the hopper motor
   * @param kickerSpeed speed for the kicker motor
   */
  public void startAllMotors(double hopperSpeed, double kickerSpeed) {
    m_indexerIO.setSpeed(hopperSpeed);
    m_kickerIO.setSpeed(kickerSpeed);
  }

  public void stopAllMotors() {
    m_indexerIO.setSpeed(0);
    m_kickerIO.setSpeed(0);
  }

  // Index = hopper sorry for confusion
  public void setHopperSpeed(double speed) {
    m_indexerIO.setSpeed(speed);
  }

  public void setKickerMotorSpeed(double speed) {
    m_kickerIO.setSpeed(speed);
  }
}
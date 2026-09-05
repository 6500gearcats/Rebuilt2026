// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotStateMachine.RobotState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private final RobotStateMachine m_RobotStateMachine;
  private final Timer m_healthTimer = new Timer();

  public Robot() {
    DataLogManager.start();
    DataLogManager.logConsoleOutput(true);
    m_robotContainer = new RobotContainer();
    m_RobotStateMachine = RobotStateMachine.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);
    m_healthTimer.start();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_RobotStateMachine.periodic();
    if (m_healthTimer.advanceIfElapsed(0.1)) {
      SmartDashboard.putNumber("Robot/BatteryVoltageV", RobotController.getBatteryVoltage());
      SmartDashboard.putNumber("Robot/CANBusUtilizationPct",
          RobotController.getCANStatus().percentBusUtilization * 100.0);
      SmartDashboard.putBoolean("Robot/RSLStatus", RobotController.getRSLState());
    }
  }

  @Override
  public void disabledInit() {
    m_robotContainer.disableInitCode();
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    m_robotContainer.disableExitCode();
  }

  @Override
  public void autonomousInit() {
    // m_robotContainer.setRobotOrientation();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
    // SignalLogger.start();
    m_RobotStateMachine.setState(RobotState.ACTIVE);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    // SignalLogger.stop();
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
    if (!m_RobotStateMachine.hasData()) {
      m_RobotStateMachine.setGameData(DriverStation.getGameSpecificMessage());
      if (!m_RobotStateMachine.getGameData().isEmpty()) {
        switch (m_RobotStateMachine.getGameData().charAt(0)) {
          case 'B':
            if (m_RobotStateMachine.getAlliance().equals(Alliance.Blue)) {
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            } else {
              m_RobotStateMachine.setState(RobotState.ACTIVE);
            }
            break;
          case 'R':
            if (m_RobotStateMachine.getAlliance().equals(Alliance.Red)) {
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            } else {
              m_RobotStateMachine.setState(RobotState.ACTIVE);
            }
            break;
        }
      }
    }
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    // SignalLogger.stop();

  }

  @Override
  public void simulationPeriodic() {
  }
}

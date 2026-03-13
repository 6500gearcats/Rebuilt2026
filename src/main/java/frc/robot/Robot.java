// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotStateMachine.RobotState;

public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  private Timer m_gcTimer = new Timer();
  private final RobotContainer m_robotContainer;
  private final RobotStateMachine m_RobotStateMachine;

  public Robot() {
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);

    m_RobotStateMachine = RobotStateMachine.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);
    if (m_gcTimer.advanceIfElapsed(5)) {
      System.gc();
    }

    if (isReal()) {
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    } else if (Constants.LoggingConstants.REPLAY) {
      setUseTiming(false); // Run as fast as possible
      String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
      Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    } else {
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      Logger.addDataReceiver(new WPILOGWriter("logs")); // Log to local folder
    }
    Logger.start();

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_RobotStateMachine.periodic();
    if (m_gcTimer.advanceIfElapsed(0.1)) {
      System.gc();
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
    m_RobotStateMachine.setState(RobotState.ACTIVE);
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
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
  }

  @Override
  public void simulationPeriodic() {
  }
}

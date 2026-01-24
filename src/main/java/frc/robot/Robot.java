// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.RobotStateMachine.RobotState;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final RobotStateMachine m_RobotStateMachine;

  public Robot() {
    m_robotContainer = new RobotContainer();
    m_RobotStateMachine = RobotStateMachine.getInstance();
    PortForwarder.add(5800, "photonvision.local", 5800);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    m_RobotStateMachine.periodic();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
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
    if(!m_RobotStateMachine.hasData()){
      m_RobotStateMachine.setGameData(DriverStation.getGameSpecificMessage());
      if(!m_RobotStateMachine.getGameData().isEmpty()){
        switch(m_RobotStateMachine.getGameData().charAt(0)){
          case 'B' :
            if(DriverStation.getAlliance().toString().equals("Blue")){
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            }
            else{
              m_RobotStateMachine.setState(RobotState.ACTIVE);
            }
            break;
          case 'R' :
            if(DriverStation.getAlliance().toString().equals("Red")){
              m_RobotStateMachine.setState(RobotState.INACTIVE);
            }
            else{
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

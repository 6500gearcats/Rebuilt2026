// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;

public class RangeFinder extends SubsystemBase {
  private RobotStateMachine robot;
  private InterpolatingDoubleTreeMap m_map;
  /** Creates a new RangeFinder. */
  public RangeFinder() {
    m_map = new InterpolatingDoubleTreeMap();
    robot = RobotStateMachine.getInstance();
    //its programmings fault
  }

  @Override
  public void periodic() {
    
    //robot.getPose();
    //SmartDashboard.putData("Distance to Hub", )
  }

  public double getShooterSpeed(double distance){
    return m_map.get(distance);
  }
}

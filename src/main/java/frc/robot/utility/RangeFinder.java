// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;

public class RangeFinder extends SubsystemBase {
  private InterpolatingDoubleTreeMap m_map;
  /** Creates a new RangeFinder. */
  public RangeFinder() {
    m_map = new InterpolatingDoubleTreeMap();
    m_map.put(2.467, 4.75);

    m_map.put(2.732, 4.75);

    m_map.put(3.017, 4.79);

    m_map.put(3.785, 5.2);

    m_map.put(5.005, 5.6);

    m_map.put(4.679, 5.7);

    m_map.put(5.250, 6.4);

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

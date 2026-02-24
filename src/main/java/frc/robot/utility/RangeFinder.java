// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;

public class RangeFinder {
  private static InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();

  static {
    m_map.put(1.43, 2.5);
m_map.put(1.73, 2.65);
m_map.put(1.8, 2.68);
m_map.put(2.01, 2.8);
m_map.put(2.2, 2.85);
m_map.put(2.48, 2.875);
m_map.put(2.63, 2.9);
m_map.put(2.75, 3.0);
m_map.put(3.03, 3.35);
m_map.put(3.15, 3.375);
m_map.put(3.37, 3.4);
m_map.put(3.61, 3.5);
m_map.put(3.77, 3.6);
m_map.put(3.99, 3.75);
m_map.put(4.4, 5.00);
 

  }
  

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

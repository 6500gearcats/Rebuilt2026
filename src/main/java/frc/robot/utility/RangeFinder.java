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
    m_map.put(2.23, 55.0);
    m_map.put(1.8, 50.0);
    m_map.put(2.0, 53.0);
    m_map.put(2.5, 60.0);
    m_map.put(2.8, 62.0);
    m_map.put(3.0, 63.0);
    m_map.put(3.4, 68.0);
    m_map.put(3.2, 64.0);
    m_map.put(3.6, 78.0);
    m_map.put(3.5, 74.0);
    m_map.put(4.0, 80.0);
    m_map.put(4.2, 86.0);
    m_map.put(5.2, 105.0);
  }

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

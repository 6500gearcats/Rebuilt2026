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
    m_map.put(2.23, 50.0);
    m_map.put(1.8, 45.0);
    m_map.put(2.0, 48.0);
    m_map.put(2.5, 55.0);
    m_map.put(2.8, 57.0);
    m_map.put(3.0, 58.0);
    m_map.put(3.4, 63.0);
    m_map.put(3.2, 61.0);
    m_map.put(3.6, 73.0);
    m_map.put(3.5, 69.0);
    m_map.put(4.0, 75.0);
    m_map.put(4.2, 81.0);
    m_map.put(5.2, 100.0);
  }

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

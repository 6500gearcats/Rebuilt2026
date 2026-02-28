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
    m_map.put(1.4, 2.5);
    m_map.put(1.75, 2.7);
    m_map.put(1.95, 2.8);
    m_map.put(2.25, 2.9);
    m_map.put(2.43, 3.0);
    m_map.put(2.62, 3.1);
    m_map.put(2.86, 3.2);
    m_map.put(2.93, 3.3);
    m_map.put(3.05, 3.4);
    m_map.put(3.23, 3.45);
    m_map.put(3.46, 3.6);
    m_map.put(3.66, 3.8);
    m_map.put(3.84, 3.9);
    m_map.put(4.04, 4.2);
    m_map.put(4.13, 4.5);
    m_map.put(4.26, 5.2);
  }

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

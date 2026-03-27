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
    m_map.put(1.5, 49.0);
    m_map.put(1.9, 51.0);
    m_map.put(2.4, 55.0);
    m_map.put(2.2, 53.0);
    m_map.put(2.75, 58.0);
    m_map.put(3.18, 65.0);
    m_map.put(3.0, 63.0);
    m_map.put(3.4, 65.0);
    m_map.put(3.79, 67.0);
    m_map.put(4.18, 70.0);
    m_map.put(4.66, 77.0);
    m_map.put(5.18, 80.0);
  }

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

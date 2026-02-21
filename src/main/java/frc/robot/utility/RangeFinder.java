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
    m_map.put(2.18, 4.5);

    m_map.put(2.27, 4.55);

    m_map.put(2.32, 4.575);

    m_map.put(2.55, 4.65);

    m_map.put(2.78, 4.75);

    m_map.put(2.92, 4.85);

    m_map.put(3.12, 5.05);

    m_map.put(3.35, 5.2);

    m_map.put(3.43, 5.3);

    m_map.put(3.57, 5.35);

    m_map.put(3.7, 5.5);

    m_map.put(3.85, 5.55);

    m_map.put(4.11, 5.6);

    m_map.put(4.28, 5.65);

    m_map.put(4.5, 6.0);

    m_map.put(4.71, 6.35);

    m_map.put(4.95, 6.5);
  }
  

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

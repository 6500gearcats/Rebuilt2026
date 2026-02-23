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
    m_map.put(1.56, 4.45);

    m_map.put(1.74, 4.5);

    m_map.put(1.96, 4.8);

    m_map.put(2.44, 5.0);

    m_map.put(2.66, 5.1);

    m_map.put(2.75, 5.2);

    m_map.put(2.96, 6.15);

    m_map.put(3.5, 6.1);

    m_map.put(3.85, 6.7);
  }
  

  public static double getShotVelocity(double distance) {
    return m_map.get(distance);
  }
}

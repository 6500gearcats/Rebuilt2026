// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;

public class RangeFinder {
  private static InterpolatingDoubleTreeMap m_shootMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap m_TOFMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap m_rotMap = new InterpolatingDoubleTreeMap();

  static {
    m_shootMap.put(1.6, 44.0);
    m_shootMap.put(1.8, 46.25);
    m_shootMap.put(2.0, 47.0);
    m_shootMap.put(2.2, 48.5);
    m_shootMap.put(2.38, 50.0);
    m_shootMap.put(2.64, 51.0);
    m_shootMap.put(3.1, 55.0);
    m_shootMap.put(3.4, 60.0);
    m_shootMap.put(3.6, 63.0);
    m_shootMap.put(3.8, 64.5);
    m_shootMap.put(4.0, 70.0);
    m_shootMap.put(4.7, 80.0);
    m_shootMap.put(5.3, 91.0);

    m_rotMap.put(-90.0, 7.0);
    m_rotMap.put(-45.0, 5.0);
    // m_rotMap.put(0.0, 0.0);
    m_rotMap.put(45.0, 5.0);
    m_rotMap.put(90.0, 7.0);

    // ! Fake values
    m_TOFMap.put(1.8, 0.85);
    m_TOFMap.put(2.5, 0.92);
    m_TOFMap.put(3.0, 0.97);
    m_TOFMap.put(3.5, 1.02);
    m_TOFMap.put(4.2, 1.08);
    m_TOFMap.put(5.2, 1.14);
  }

  public static double getShotVelocity(double distance) {
    return m_shootMap.get(distance);
  }

  public static double getTOF(double distance) {
    return m_TOFMap.get(distance);
  }

  public static double getRotAdder(double deg) {
    // if (Math.abs(deg) > 90) {
    // deg = 90 * (Math.abs(deg) / deg);
    // }
    // return m_rotMap.get(deg);

    return (0.000725312 * (deg * deg) + 2.82988);
  }
}

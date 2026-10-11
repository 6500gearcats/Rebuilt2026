// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class RangeFinder {
  private static InterpolatingDoubleTreeMap m_shootMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap m_TOFMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();
  private static InterpolatingDoubleTreeMap m_rotMap = new InterpolatingDoubleTreeMap();

  static {

    m_shootMap.put(1.7845524, 47.1966146);
    m_shootMap.put(2.21180044, 50.8945313);
    m_shootMap.put(2.44133151, 55.1516544);
    m_shootMap.put(2.85586734, 57.5031467);
    m_shootMap.put(3.22893898, 58.3346354);
    m_shootMap.put(3.57005063, 57.6749132);
    m_shootMap.put(4.10740451, 61.0183377);
    m_shootMap.put(4.46538443, 64.5872396);
    m_shootMap.put(4.92015014, 56.7465278);

    m_hoodMap.put(1.7845524, 0.01235623);
    m_hoodMap.put(2.21180044, 0.075927734);
    m_hoodMap.put(2.44133151, 0.02648208);
    m_hoodMap.put(2.85586734, 0.08269586);
    m_hoodMap.put(3.22893898, 0.12628852);
    m_hoodMap.put(3.57005063, 0.24892849);
    m_hoodMap.put(4.10740451, 0.29673937);
    m_hoodMap.put(4.46538443, 0.32543945);
    m_hoodMap.put(4.92015014, 0.34509277);

    // ! Fake values
    m_TOFMap.put(1.7845524, 0.7759643441);
    m_TOFMap.put(2.21180044, 0.8953730721);
    m_TOFMap.put(2.44133151, 1.01208837);
    m_TOFMap.put(2.85586734, 1.106016396);
    m_TOFMap.put(3.22893898, 1.077115465);
    m_TOFMap.put(3.57005063, 0.9879116298);
    m_TOFMap.put(4.10740451, 1.038766153);
    m_TOFMap.put(4.46538443, 1.12867541);
    m_TOFMap.put(4.92015014, 1.161381345);

    m_rotMap.put(-90.0, 7.0);
    m_rotMap.put(-45.0, 5.0);
    // m_rotMap.put(0.0, 0.0);
    m_rotMap.put(45.0, 5.0);
    m_rotMap.put(90.0, 7.0);
  }

  public static double getShotVelocity(double distance) {
    return m_shootMap.get(distance);
  }

  public static double getHoodRotations(double distance) {
    return m_hoodMap.get(distance);
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

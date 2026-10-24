// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class RangeFinder {
  private RangeFinder() {}
  private static final InterpolatingDoubleTreeMap m_shootMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_TOFMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_hoodMap = new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap m_rotMap = new InterpolatingDoubleTreeMap();

  static {

    m_shootMap.put(1.7845524, 47.1966146);
    m_shootMap.put(2.21180044, 50.8945313);
    m_shootMap.put(2.44133151, 55.1516544);
    m_shootMap.put(2.85586734, 57.3031467);
    m_shootMap.put(3.22893898, 57.5346354);
    m_shootMap.put(3.57005063, 59.6749132);
    m_shootMap.put(4.10740451, 61.0183377);
    m_shootMap.put(4.46538443, 64.5872396);
    m_shootMap.put(4.92015014, 65.7465278);
    m_shootMap.put(5.22015014, 70.7465278);
    m_shootMap.put(8.6015014, 80.0);

//    2 rpm Slower
//    m_shootMap.put(1.7845524, 45.1966146);
//    m_shootMap.put(2.21180044, 48.8945313);
//    m_shootMap.put(2.44133151, 53.1516544);
//    m_shootMap.put(2.85586734, 55.3031467);
//    m_shootMap.put(3.22893898, 55.5346354);
//    m_shootMap.put(3.57005063, 57.6749132);
//    m_shootMap.put(4.10740451, 59.0183377);
//    m_shootMap.put(4.46538443, 62.5872396);
//    m_shootMap.put(4.92015014, 63.7465278);
//    m_shootMap.put(5.22015014, 68.7465278);
//    m_shootMap.put(8.6015014, 80.0);

    m_hoodMap.put(1.4845524, 0.0);
    m_hoodMap.put(1.7845524, 0.01235623);
    m_hoodMap.put(2.21180044, 0.075927734);
    m_hoodMap.put(2.44133151, 0.02648208);
    m_hoodMap.put(2.85586734, 0.08269586);
    m_hoodMap.put(3.22893898, 0.12628852);
    m_hoodMap.put(3.57005063, 0.24892849);
    m_hoodMap.put(4.10740451, 0.29673937);
    m_hoodMap.put(4.46538443, 0.32543945);
    m_hoodMap.put(4.92015014, 0.34509277);
    m_hoodMap.put(5.22015014, 0.34509277);
    m_hoodMap.put(5.6015014, 0.55509277);

    m_TOFMap.put(1.7845524, 0.7759643441);
    m_TOFMap.put(2.21180044, 0.8953730721);
    m_TOFMap.put(2.44133151, 1.01208837);
    m_TOFMap.put(2.85586734, 1.106016396);
    m_TOFMap.put(3.22893898, 1.077115465);
    m_TOFMap.put(3.57005063, 0.9879116298);
    m_TOFMap.put(4.10740451, 1.038766153);
    m_TOFMap.put(4.46538443, 1.12867541);
    m_TOFMap.put(4.92015014, 1.161381345);

    m_rotMap.put(-110.0, 6.0);
    m_rotMap.put(-90.0, 4.0);
    m_rotMap.put(-45.0, 2.0);
    m_rotMap.put(0.0, 0.0);
    m_rotMap.put(45.0, 2.0);
    m_rotMap.put(90.0, 4.0);
    m_rotMap.put(110.0, 6.0);
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
      return m_rotMap.get(deg);
    // return (0.000725312 * (deg * deg) + 2.82988);
  }
}

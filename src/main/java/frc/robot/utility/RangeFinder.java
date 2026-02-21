// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utility;

import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotStateMachine;
import frc.robot.Constants.TurretConstants;

public class RangeFinder {
  private static InterpolatingDoubleTreeMap m_map = new InterpolatingDoubleTreeMap();

  static {
    m_map.put(2.467, 4.75);

    m_map.put(2.732, 4.75);

    m_map.put(3.017, 4.79);

    m_map.put(2.257, 4.85);

    m_map.put(3.496, 5.0);

    m_map.put(3.644, 5.25);

    m_map.put(3.785, 5.2);

    m_map.put(4.310, 5.35);

    m_map.put(5.005, 5.6);

    m_map.put(4.679, 5.7);

    m_map.put(5.250, 6.4);

    m_map.put(4.864, 6.8);
  }

  public static double getShotRPM(double distance) {
    return m_map.get(distance);
  }

  // tangential velocity = angular velocity * radius
  public static LinearVelocity getShotTangentialVelocity(double distance) {
    double rpm = getShotRPM(distance);
    return LinearVelocity.ofBaseUnits(rpm * TurretConstants.kShooterWheelRadius, MetersPerSecond);
  }
}

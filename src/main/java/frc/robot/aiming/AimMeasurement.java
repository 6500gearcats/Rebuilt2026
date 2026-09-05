package frc.robot.aiming;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

public record AimMeasurement(
      Distance distance,
      Rotation2d pitch,
      double shooterControl,
      Time time) {
}

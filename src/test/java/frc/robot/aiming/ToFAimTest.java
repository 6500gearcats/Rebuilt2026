// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aiming;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import java.util.List;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.aiming.AimParams.AimStatus;

/**
 * Unit tests for {@link ToFAim#update}. Added 2026-09-09 — see
 * {@code plans/review_plan.md} R6-2.
 *
 * <p>Every calibration table here uses two points (2 m and 4 m) so
 * {@code InterpolatingDoubleTreeMap} has a real range to interpolate/clamp within. All
 * shooter poses are placed at the origin and targets along +X, so straight-line distance is
 * just the target's X coordinate — this keeps expected values easy to hand-verify rather than
 * requiring trigonometry to sanity-check the test itself.
 */
class ToFAimTest {

  private static final double EPS = 1e-6;

  /** distance=2m -> pitch=30deg, speed=50, tof=0.4s. distance=4m -> pitch=45deg, speed=70, tof=0.6s. */
  private static List<AimMeasurement> calibrationTable() {
    return List.of(
        new AimMeasurement(Meters.of(2), Rotation2d.fromDegrees(30), 50.0, Seconds.of(0.4)),
        new AimMeasurement(Meters.of(4), Rotation2d.fromDegrees(45), 70.0, Seconds.of(0.6)));
  }

  private static Pose3d targetAt(double xMeters) {
    return new Pose3d(new Translation3d(xMeters, 0, 0), new edu.wpi.first.math.geometry.Rotation3d());
  }

  /**
   * With zero shooter velocity, {@code afterShooting} never moves away from the shooter's own
   * position between iterations ({@code start.plus(velocity.times(tof))} is just {@code start}
   * regardless of {@code tof}), so the convergence loop's very first iteration already
   * satisfies {@code error < EPSILON} — every further iteration would recompute the identical
   * value. The output should exactly match the calibration table's 2 m entry.
   */
  @Test
  void stationaryRobotConvergesToDirectTableLookup() {
    AimConstraints wideOpen =
        new AimConstraints(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), 100.0);
    ToFAim aim = new ToFAim(calibrationTable(), wideOpen);

    AimParams params = aim.update(targetAt(2.0), new Pose3d(), Translation2d.kZero);

    assertEquals(AimStatus.Possible, params.status);
    assertEquals(30.0, params.pitch.getDegrees(), EPS);
    assertEquals(50.0, params.output, EPS);
    assertEquals(0.4, params.tof, EPS);
    // Target is due +X of the shooter with zero lead, so yaw is 0.
    assertEquals(0.0, params.yaw.getDegrees(), EPS);
  }

  /**
   * A target far beyond the calibration table's range (100 m vs. a 4 m max entry).
   * {@code InterpolatingDoubleTreeMap} does not throw or signal out-of-range — it clamps to
   * the nearest known point (here, the 4 m entry: speed=70). This test's {@link AimConstraints}
   * caps {@code maxOutput} at 60, below that clamped value, so the shot is correctly rejected
   * — demonstrating that "unreachable range" surfaces as a constraint violation in this
   * implementation, not as a distinct status.
   */
  @Test
  void distanceBeyondCalibrationRangeClampsThenFailsConstraints() {
    AimConstraints capped =
        new AimConstraints(Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(90), 60.0);
    ToFAim aim = new ToFAim(calibrationTable(), capped);

    AimParams params = aim.update(targetAt(100.0), new Pose3d(), Translation2d.kZero);

    assertEquals(AimStatus.Impossible, params.status);
    assertFalse(params.isOk());
  }

  /**
   * A perfectly reachable, in-range shot (2 m, matching a table entry exactly) that violates
   * the physical pitch constraint — the calibration table wants 30 degrees, but this
   * {@link AimConstraints} only allows 40-80 degrees. Isolates the "constraints reject an
   * otherwise valid geometric solution" path from the "distance clamped" path above.
   */
  @Test
  void inRangeShotStillFailsWhenPitchConstraintIsNarrower() {
    AimConstraints narrowPitch =
        new AimConstraints(Rotation2d.fromDegrees(40), Rotation2d.fromDegrees(80), 100.0);
    ToFAim aim = new ToFAim(calibrationTable(), narrowPitch);

    AimParams params = aim.update(targetAt(2.0), new Pose3d(), Translation2d.kZero);

    assertEquals(AimStatus.Impossible, params.status);
  }
}

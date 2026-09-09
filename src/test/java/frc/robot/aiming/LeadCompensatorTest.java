// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.aiming;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.aiming.AimParams.SpeedControl;

/**
 * Unit tests for {@link LeadCompensator#computeLeadTarget}. Added 2026-09-09 — see
 * {@code plans/review_plan.md} R6-3.
 *
 * <p>Uses hand-written {@link AimStrategy} stubs (lambdas) rather than a real {@link ToFAim}
 * calibration table — this isolates {@code LeadCompensator}'s own convergence/shift logic
 * from the inner strategy's math, and lets each test fix exactly the {@code tof}/status
 * behavior it needs.
 */
class LeadCompensatorTest {

  private static final double EPS = 1e-6;

  private static AimParams possibleWithTof(double tof) {
    AimParams p = new AimParams();
    p.status = AimParams.AimStatus.Possible;
    p.control = SpeedControl.MechanismControl;
    p.tof = tof;
    return p;
  }

  /**
   * With zero field velocity, {@code virtualTarget = hub - velocity*tof} degenerates to
   * exactly {@code hub}'s translation for any {@code tof}, so the loop converges on its first
   * iteration. Note: a converged result's rotation is unconditionally forced to
   * {@link Rotation3d#kZero} by {@code computeLeadTarget}'s own construction of {@code next},
   * regardless of the input {@code hubPose}'s rotation — so this test constructs
   * {@code hubPose} with zero rotation from the start, to make "returns hubPose exactly" a
   * claim that's actually true rather than one that happens to ignore a rotation mismatch.
   */
  @Test
  void zeroVelocityConvergesToHubTranslationOnFirstIteration() {
    Pose3d hub = new Pose3d(new Translation3d(5, 3, 2), new Rotation3d());
    Pose3d shooter = new Pose3d();
    AimStrategy fixedTof = (target, s, v) -> possibleWithTof(0.5);

    Pose3d result = LeadCompensator.computeLeadTarget(hub, shooter, Translation2d.kZero, fixedTof);

    assertEquals(hub.getX(), result.getX(), EPS);
    assertEquals(hub.getY(), result.getY(), EPS);
    assertEquals(hub.getZ(), result.getZ(), EPS);
  }

  /**
   * A robot moving in +X at 2 m/s with a constant tof=0.5s stub strategy: the virtual target
   * shifts by {@code -velocity*tof = (-1, 0)} from the real hub at (10, 0, 2), landing at
   * (9, 0, 2) — opposite the direction of travel. Because the stub returns the same tof
   * regardless of input pose, {@code next} is identical on every iteration once first
   * computed, so convergence happens on the second iteration (first: shift from the initial
   * {@code best=hub}; second: confirm no further movement).
   */
  @Test
  void movingRobotShiftsVirtualTargetOppositeTravelDirection() {
    Pose3d hub = new Pose3d(new Translation3d(10, 0, 2), new Rotation3d());
    Pose3d shooter = new Pose3d();
    Translation2d velocity = new Translation2d(2, 0); // moving toward +X at 2 m/s
    AimStrategy fixedTof = (target, s, v) -> possibleWithTof(0.5);

    Pose3d result = LeadCompensator.computeLeadTarget(hub, shooter, velocity, fixedTof);

    assertEquals(9.0, result.getX(), EPS);
    assertEquals(0.0, result.getY(), EPS);
    assertEquals(2.0, result.getZ(), EPS);
  }

  /**
   * When the inner strategy reports {@link AimParams.AimStatus#Impossible} on the very first
   * call, the loop must break immediately without ever constructing a {@code next} candidate
   * — returning {@code best} in its untouched initial state, i.e. {@code hubPose} exactly
   * (translation <em>and</em> rotation, since no reassignment ever happened).
   */
  @Test
  void impossibleInnerStrategyBreaksImmediatelyAndReturnsHubUnchanged() {
    Pose3d hub = new Pose3d(new Translation3d(7, -2, 1), new Rotation3d(0, 0, 0.3));
    Pose3d shooter = new Pose3d();
    AimStrategy alwaysImpossible = (target, s, v) -> AimParams.impossible();

    Pose3d result =
        LeadCompensator.computeLeadTarget(hub, shooter, new Translation2d(1, 1), alwaysImpossible);

    assertEquals(hub, result);
  }
}

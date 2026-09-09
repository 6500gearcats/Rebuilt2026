// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Unit tests for {@link Turret#findCC}, the closest-congruent-angle algorithm used to resolve
 * multi-turn wrap-around without hardware. Added 2026-09-09 — see
 * {@code plans/review_plan.md} R6-1.
 *
 * <p>{@code findCC} was already made package-private {@code static} specifically so it could
 * be unit-tested independently of hardware (see its own Javadoc) — this file is the first use
 * of that seam. Every expected value below was hand-traced against the actual algorithm before
 * being written as an assertion, not derived from what the code happens to currently return.
 *
 * <p>Floating-point comparisons use a {@code 1e-9} delta rather than exact equality — every
 * value here is built from sums/differences of numbers with exact binary representations
 * (0.5, 1.0, 2.0, ...), so in practice the delta is not load-bearing, but it's the correct
 * habit for any {@code double} assertion.
 */
class TurretTest {

  private static final double EPS = 1e-9;

  /**
   * When the target reference already equals the current position, no wrapping or walking is
   * needed — {@code findCC} should return it unchanged.
   */
  @Test
  void sameRotationNoWrapNeeded() {
    assertEquals(0.3, Turret.findCC(0.3, 0.3, -0.5, 0.5), EPS);
  }

  /**
   * A small move within tolerance of the current position: {@code error < 0.5} is satisfied
   * immediately, so the reference is returned as-is without entering the walk loop.
   */
  @Test
  void smallAdjustmentNoWrap() {
    assertEquals(0.4, Turret.findCC(0.2, 0.4, -0.5, 0.5), EPS);
  }

  /**
   * Multi-turn case: the turret is at 2.3 rotations: the target orientation 0.3 (congruent to
   * ..., -1.7, 0.3, 1.3, 2.3, ...) is far from 2.3 in its raw form, so the walk loop must step
   * forward by whole rotations (0.3 -&gt; 1.3 -&gt; 2.3) until it lands within 0.5 of the
   * current position — landing exactly on 2.3 here (error 0).
   */
  @Test
  void wrapsPositiveTowardCurrentPosition() {
    assertEquals(2.3, Turret.findCC(2.3, 0.3, -3, 3), EPS);
  }

  /**
   * Same idea in the negative direction: current position -2.3, target orientation 0.3. The
   * walk steps backward (0.3 -&gt; -0.7 -&gt; -1.7 -&gt; -2.7) until landing within 0.5 of -2.3,
   * which first happens at -2.7 (error 0.4) — -1.7 is only 0.6 away, not yet close enough.
   */
  @Test
  void wrapsNegativeTowardCurrentPosition() {
    assertEquals(-2.7, Turret.findCC(-2.3, 0.3, -3, 3), EPS);
  }

  /**
   * A reference whose fractional part cannot be wrapped into a narrow [min, max] window at
   * all (0.9 shifted by whole rotations only ever lands on ..., -1.1, -0.1, 0.9, ... — none of
   * which fall inside [0.0, 0.3]) must clamp to {@code max}, regardless of the current
   * position, per the "still out of range even after wrapping" branch.
   */
  @Test
  void clampsToMaxWhenWrappingCannotReachRange() {
    assertEquals(0.3, Turret.findCC(0.0, 0.9, 0.0, 0.3), EPS);
    assertEquals(0.3, Turret.findCC(100.0, 0.9, 0.0, 0.3), EPS); // position is irrelevant here
  }

  /**
   * Boundary behavior at exactly {@code error == 0.5}: the check is strictly {@code < 0.5}, so
   * a reference sitting exactly half a rotation from every congruent candidate never satisfies
   * either the immediate-return check or the walk loop's acceptance check. Every candidate
   * (0.5, 1.5, 2.5, 3.5, ...) is exactly 0.5 away from position 1.0; the walk keeps advancing
   * until the next candidate (3.5) would exceed {@code max} (3), at which point the loop
   * breaks and returns the last value it had accepted (2.5) — not the value that failed the
   * bounds check.
   */
  @Test
  void exactHalfRotationBoundaryNeverCountsAsClose() {
    assertEquals(2.5, Turret.findCC(1.0, 0.5, -3, 3), EPS);
  }
}

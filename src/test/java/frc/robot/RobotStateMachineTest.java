// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.RobotStateMachine.FieldZone;

/**
 * Unit tests for {@link RobotStateMachine#checkZone}. Added 2026-09-09 — see
 * {@code plans/review_plan.md} R6-4.
 *
 * <p>{@code RobotStateMachine} is an eagerly-initialized singleton with a private constructor
 * — these tests drive it through the shared {@link RobotStateMachine#getInstance()} instance
 * via {@link RobotStateMachine#setPose} rather than constructing a fresh one per test. Alliance
 * is controlled via {@link DriverStationSim#setAllianceStationId}, which requires
 * {@link HAL#initialize} first; both APIs verified against WPILib 2026.2.1 sources before use.
 */
class RobotStateMachineTest {

  @BeforeAll
  static void initializeHal() {
    assert HAL.initialize(500, 0);
  }

  private final RobotStateMachine rsm = RobotStateMachine.getInstance();

  @BeforeEach
  void setBlueAlliance() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
  }

  @Test
  void blueAllianceZoneBoundaries() {
    // x < 5.4 is ALLIANCE for Blue.
    rsm.setPose(new Pose2d(5.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.ALLIANCE, rsm.checkZone());

    // x > 11.0 is OPPONENT for Blue.
    rsm.setPose(new Pose2d(11.5, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.OPPONENT, rsm.checkZone());
  }

  @Test
  void redAllianceMirrorsTheSameBoundaries() {
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.notifyNewData();

    // x < 5.4 is OPPONENT for Red (mirrored from Blue's ALLIANCE zone).
    rsm.setPose(new Pose2d(5.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.OPPONENT, rsm.checkZone());

    // x > 11.0 is ALLIANCE for Red.
    rsm.setPose(new Pose2d(11.5, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.ALLIANCE, rsm.checkZone());
  }

  /**
   * <b>Found while writing this test, not a design choice made here:</b>
   * {@code checkZone()}'s Y-boundary code and the {@link FieldZone} enum's own Javadoc
   * directly contradict each other. The enum documents {@code NEUTRAL_TOP} as
   * "y &gt; 4.2 m" and {@code NEUTRAL_BOTTOM} as "y &lt; 3.8 m" — but the code below returns
   * {@code NEUTRAL_BOTTOM} for {@code y > 4.2} and {@code NEUTRAL_TOP} for {@code y < 3.8}:
   * exactly swapped. This test asserts what {@code checkZone()} actually returns today (its
   * job is to characterize current behavior, not silently redefine it) — it does not assert
   * that this is the intended mapping. Determining which one is wrong requires the real field
   * geometry/orientation, which is outside what code alone can settle. See
   * {@code plans/review_plan.md} R6-4 and flag to the team before trusting either the enum
   * Javadoc or this test as the source of truth for which Y range is "top" vs. "bottom."
   */
  @Test
  void neutralZoneYBoundaries_currentBehaviorTopBottomSwappedVsEnumJavadoc() {
    // Neutral band: 5.4 <= x <= 11.0.
    rsm.setPose(new Pose2d(8.0, 4.5, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.NEUTRAL_BOTTOM, rsm.checkZone()); // y=4.5 > 4.2

    rsm.setPose(new Pose2d(8.0, 4.0, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.NEUTRAL_CENTER, rsm.checkZone()); // y=4.0 in [3.8, 4.2]

    rsm.setPose(new Pose2d(8.0, 3.5, new edu.wpi.first.math.geometry.Rotation2d()));
    assertEquals(FieldZone.NEUTRAL_TOP, rsm.checkZone()); // y=3.5 < 3.8
  }
}

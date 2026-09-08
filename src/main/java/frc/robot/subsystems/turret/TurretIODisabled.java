package frc.robot.subsystems.turret;

import edu.wpi.first.units.measure.Angle;

/**
 * No-op implementation of {@link TurretIO} used when the turret is physically absent
 * or intentionally disabled (e.g., a robot build that doesn't have a turret installed yet,
 * or integration tests where turret motion would interfere).
 *
 * <p>All methods are empty — no CAN traffic is generated, and inputs are never updated.
 * The {@link frc.robot.subsystems.turret.Turret} subsystem will continue to function at
 * the software level (commands will schedule and complete) but the physical turret will
 * not move and {@code TurretIOInputs.motorConnected} will remain {@code false}.
 *
 * <p>Select this implementation in {@link frc.robot.RobotContainer} when the turret is
 * not installed rather than guarding every turret call site with a null check.
 */
public class TurretIODisabled implements TurretIO {
  /** No-op — calibration not required without hardware. */
  public void calibrate() {}

  /** No-op — ignored without hardware. */
  public void setPosition(Angle angle) {}

  /** No-op — inputs are left at their default (disconnected) values. */
  public void updateInputs(TurretIOInputs inputs) {}
}

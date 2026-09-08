package frc.robot.subsystems.drivetrain;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * Stub holder for WPILib SysId characterization routines.
 *
 * <p>The original Flywheel and Turret SysId routines were removed in Stage 5 (refactor).
 * This class exists so that {@link frc.robot.RobotContainer} can reference {@code SysIDUtil}
 * without null-check ceremony — all methods return safe no-ops.
 *
 * <p>Re-implement for the Shooter and new Turret in Stage 8 when the physical robot
 * is available for characterization runs.
 */
public class SysIDUtil {
  public SysIDUtil() {}

  /**
   * Returns all SysId characterization commands in sequence.
   *
   * @return empty — re-implement in Stage 8.
   */
  public Optional<SequentialCommandGroup> sysIdAll() {
    return Optional.empty();
  }

  /**
   * Returns a quasistatic SysId command in the given direction.
   *
   * @param direction Forward or Reverse.
   * @return a no-op command — re-implement in Stage 8.
   */
  public Command sysIdQuasistatic(Direction direction) {
    return Commands.none();
  }

  /**
   * Returns a dynamic SysId command in the given direction.
   *
   * @param direction Forward or Reverse.
   * @return a no-op command — re-implement in Stage 8.
   */
  public Command sysIdDynamic(Direction direction) {
    return Commands.none();
  }

  /**
   * Returns {@code false} — indicates that no SysId routines are currently wired up.
   * {@link frc.robot.RobotContainer} uses this to gate SysId controller bindings.
   */
  public boolean isPresent() {
    return false;
  }
}

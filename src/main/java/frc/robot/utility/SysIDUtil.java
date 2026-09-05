package frc.robot.utility;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

/**
 * SysID helper — Flywheel and old Turret routines removed in Stage 5.
 * Re-implement for Shooter and new Turret in Stage 8.
 */
public class SysIDUtil {
  public SysIDUtil() {}

  public Optional<SequentialCommandGroup> sysIdAll() {
    return Optional.empty();
  }

  public Command sysIdQuasistatic(Direction direction) {
    return Commands.none();
  }

  public Command sysIdDynamic(Direction direction) {
    return Commands.none();
  }

  public boolean isPresent() {
    return false;
  }
}

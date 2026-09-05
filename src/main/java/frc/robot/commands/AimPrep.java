package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.aiming.AimParams;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import java.util.function.Supplier;

/**
 * Parallel turret tracking + shooter spin-up command.
 * Stage 4 stub — wired to RobotStateMachine in Stage 5.
 */
public class AimPrep {
    // TODO Stage 5: replace with RobotStateMachine-backed implementation
    public Command build(Turret turret, Shooter shooter, Supplier<AimParams> params) {
        return Commands.parallel(
            shooter.shoot(params)
        );
    }
}

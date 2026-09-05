package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.Hopper;
import java.util.function.BooleanSupplier;

/**
 * Waits until shoot-ready conditions are met, then triggers the hopper.
 * Stage 4 stub — wired to RobotStateMachine.isShootReady() in Stage 5.
 */
public class ShootWhenReady {
    // TODO Stage 5: replace with RobotStateMachine-backed implementation
    public Command build(Hopper hopper, BooleanSupplier shootReady) {
        return Commands.sequence(
            Commands.waitUntil(shootReady),
            hopper.runOnce(() -> hopper.startAllMotors(-0.9, 1))
        );
    }
}

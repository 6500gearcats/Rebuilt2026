// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.DashboardTelemetry;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Command that drives the turret at a supplied speed.
 */
public class MoveTurret extends Command {
  private final Turret m_turret;
  private final DoubleSupplier speedSupplier;
  private final DashboardTelemetry.DebugPublisher debugTelemetry = new DashboardTelemetry.DebugPublisher();

  /**
   * Creates a new MoveTurret command.
   *
   * @param turret turret subsystem to control
   * @param speed  supplier for the desired turret speed
   */
  public MoveTurret(Turret turret, DoubleSupplier speed) {
    m_turret = turret;
    this.speedSupplier = speed;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = speedSupplier.getAsDouble();
    m_turret.setSpeed(speed);
    if (debugTelemetry.shouldPublish()) {
      SmartDashboard.putNumber("Debug/Turret/Manual Duty Cycle", speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_turret.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

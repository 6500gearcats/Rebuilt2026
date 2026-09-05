// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTurretToHub extends Command {
  /** Creates a new AlignTurretToHub. */
  private double kP = 0.05;
  private double kI = 0;
  private double kD = 0;

  private PIDController pid = new PIDController(kP, kI, kD);
  private Turret m_turret;
  private RobotStateMachine m_StateMachine = RobotStateMachine.getInstance();

  private Pose2d prevPose = new Pose2d();
  private double prevTurretRot = 0;
  private final Timer m_telemetryTimer = new Timer();

  public AlignTurretToHub(Turret turret) {
    m_turret = turret;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {
    m_telemetryTimer.restart();
  }

  @Override
  public void execute() {
    Pose2d currPose = m_StateMachine.getTurretPose();
    Pose2d m_targetPose = m_StateMachine.getTargetPose();

    Translation2d robotToTarget = m_targetPose.getTranslation()
        .minus(currPose.getTranslation());
    Rotation2d turretAndRobot = currPose.getRotation()
        .plus(new Rotation2d(Math.toRadians(m_turret.getConvertedTurretPosition())));
    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(turretAndRobot);

    double newError = turretToTargetAngle.getDegrees() + m_turret.getConvertedTurretPosition();
    newError = (Math.abs(newError) - 180) * (newError / Math.abs(newError));
    if (newError > 0) {
      if (Math.abs(newError) > 110) {
        newError = 110 * (Math.abs(newError) / newError);
      }
    } else {
      if (Math.abs(newError) > 103) {
        newError = 110 * (Math.abs(newError) / newError);
      }
    }
    if (Math.abs(newError) > 0.005) {
      m_turret.setPosition(newError);
    }

    // Debug telemetry — 10 Hz
    if (m_telemetryTimer.advanceIfElapsed(0.1)) {
      Translation2d errorFromPrev = prevPose.minus(currPose).getTranslation();
      double errorFromPrevRot = prevTurretRot - m_turret.getConvertedTurretPosition();
      Pose2d newTurretPose = new Pose2d(currPose.getTranslation(), turretAndRobot);
      SmartDashboard.putNumber("Turret/AlignErrorX",          errorFromPrev.getX());
      SmartDashboard.putNumber("Turret/AlignErrorY",          errorFromPrev.getY());
      SmartDashboard.putNumber("Turret/AlignRobotRotError",   prevPose.minus(currPose).getRotation().getDegrees());
      SmartDashboard.putNumber("Turret/AlignRotError",        errorFromPrevRot);
      SmartDashboard.putNumber("Turret/TurretPlusRobotAngle", turretAndRobot.getDegrees());
      SmartDashboard.putNumber("Turret/DistToHubM",           newTurretPose.getTranslation().getDistance(m_targetPose.getTranslation()));
      SmartDashboard.putNumber("Turret/AlignmentErrorDeg",    turretToTargetAngle.getDegrees());
      SmartDashboard.putNumber("Turret/PositionSetpointDeg",  newError);
      prevPose = currPose;
      prevTurretRot = m_turret.getConvertedTurretPosition();
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
    // boolean isDone = m_turret.getAlignRate() < 0.1;
    return false;
  }
}

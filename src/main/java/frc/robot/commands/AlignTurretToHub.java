// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTurretToHub extends Command {
  /** Creates a new AlignTurretToHub. */

  private Turret m_turret;
  private RobotStateMachine m_StateMachine = RobotStateMachine.getInstance();

  private Pose2d prevPose = new Pose2d();
  private double prevTurretRot = 0;

  public AlignTurretToHub(Turret turret) {
    m_turret = turret;
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
    Pose2d currPose = m_StateMachine.getTurretPose();
    Translation2d errorFromPrev = prevPose.minus(currPose).getTranslation();
    double errorFromPrevRot = prevTurretRot - m_turret.getConvertedTurretPosition();
    SmartDashboard.putNumber("errorFromPrev.getX", errorFromPrev.getX());
    SmartDashboard.putNumber("errorFromPrev.getY", errorFromPrev.getY());
    SmartDashboard.putNumber("errorFromPrevRobotRot", prevPose.minus(currPose).getRotation().getDegrees());
    SmartDashboard.putNumber("errroFromPrevRot", errorFromPrevRot);

    Pose2d m_targetPose = m_StateMachine.getTargetPose(); // Get updating pose of target from state machine

    Translation2d robotToTarget = m_targetPose.getTranslation()
        .minus(m_StateMachine.getTurretPose().getTranslation()); // gets x and y difference between robot and april tag
    Rotation2d turretAndRobot = m_StateMachine.getTurretPose().getRotation();

    Pose2d newTurretPose = new Pose2d(m_StateMachine.getTurretPose().getTranslation(), turretAndRobot);
    SmartDashboard.putNumber("turretAndRobot", turretAndRobot.getDegrees());
    SmartDashboard.putNumber("Dist to Tag", newTurretPose.getTranslation().getDistance(m_targetPose.getTranslation()));

    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(turretAndRobot); // angle of x and y difference
                                                                                     // minue rotation between tag/robot
    SmartDashboard.putNumber("turretError", turretToTargetAngle.getDegrees());

    double newError = turretToTargetAngle.getDegrees() + m_turret.getConvertedTurretPosition();
    newError = (Math.abs(newError) - 180) * Math.signum(newError); // (newError / Math.abs(newError)); Signum handles
                                                                   // divide by zero

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
      SmartDashboard.putBoolean("Aligned", false);
      m_turret.setPosition(newError);
    }
    SmartDashboard.putBoolean("Aligned", true);

    SmartDashboard.putNumber("tunring_pos_setpoint", newError);
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

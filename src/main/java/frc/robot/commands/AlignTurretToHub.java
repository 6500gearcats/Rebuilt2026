// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
    Pose2d m_targetPose = m_StateMachine.getTargetPose(); // Get updating pose of target from state machine

    Translation2d robotToTarget = m_targetPose.getTranslation()
        .minus(m_StateMachine.getTurretPose().getTranslation()); // gets x and y difference between robot and april tag
    Rotation2d turretAndRobot = m_StateMachine.getTurretPose().getRotation()
        .plus(new Rotation2d(Math.toRadians(m_turret.getConvertedTurretPosition())));// gets rotation of motor in
                                                                                     // relation to field

    Pose2d newTurretPose = new Pose2d(m_StateMachine.getTurretPose().getTranslation(), turretAndRobot);
    SmartDashboard.putNumber("turretAndRobot", turretAndRobot.getDegrees());
    SmartDashboard.putNumber("Dist to Tag", newTurretPose.getTranslation().getDistance(m_targetPose.getTranslation()));

    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(turretAndRobot); // angle of x and y difference
                                                                                     // minue rotation between tag/robot
    SmartDashboard.putNumber("turretError", turretToTargetAngle.getDegrees());

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

    if (Math.abs(newError) > 0.01) {
      m_turret.setPosition(newError);
    }

    // double error = pid.calculate(m_turret.getConvertedTurretPosition(),
    // newError); // sets turret speed
    // m_turret.setSpeed(error);
    SmartDashboard.putNumber("tunring_pos_setpoint", newError);
    // SmartDashboard.putNumber("turretTurnRate", rate);
    // m_turret.setSpeed(rate);
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

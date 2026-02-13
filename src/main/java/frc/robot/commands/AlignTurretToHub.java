// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private Turret m_turret;
  private Pose2d m_tagpose = TurretConstants.HubPose;
  private RobotStateMachine m_StateMachine = RobotStateMachine.getInstance();

  private final StructPublisher<Pose2d> posePublisher = NetworkTableInstance.getDefault()
            .getTable("StateMachinePose")
            .getStructTopic("Hub Pose", Pose2d.struct)
            .publish();

  public AlignTurretToHub(Turret turret) {
    m_turret = turret;
    posePublisher.set(m_tagpose);
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {   //TODO: Fix turret alignment and angle measurment, currently jumping from ~-40 to 40 or vice versa
   
    Translation2d robotToTarget = m_tagpose.getTranslation().minus(m_StateMachine.getPose().getTranslation());
    Rotation2d turretAndRobot = m_StateMachine.getPose().getRotation().plus(new Rotation2d(Math.toRadians(m_turret.getConvertedTurretPosition())));
    SmartDashboard.putNumber("turretAndRobot", turretAndRobot.getDegrees());
    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(turretAndRobot);
    SmartDashboard.putNumber("turretError", turretToTargetAngle.getDegrees());
    double convertedDeg = (180 - Math.abs(turretToTargetAngle.getDegrees())) * (turretToTargetAngle.getDegrees()/Math.abs(turretToTargetAngle.getDegrees()));
    SmartDashboard.putNumber("turretConvertedError", convertedDeg);
    double rate = convertedDeg * 0.05;

    SmartDashboard.putNumber("turretTurnRate", rate);
    m_turret.setSpeed(rate);
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

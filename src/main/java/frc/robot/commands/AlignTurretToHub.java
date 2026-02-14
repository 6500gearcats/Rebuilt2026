// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.turret.Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignTurretToHub extends Command {
  /** Creates a new AlignTurretToHub. */
  private double kP = 0.001;
  private double kI = 0;
  private double kD = 0;

  private PIDController pid = new PIDController(kP, kI, kD);
  private Turret m_turret;
  private CommandSwerveDrivetrain driveTrain;
  private Pose3d m_tagpose = TurretConstants.HUB_POSE2D;
  private RobotStateMachine m_StateMachine = RobotStateMachine.getInstance();

  public AlignTurretToHub(Turret turret, CommandSwerveDrivetrain driveTrain) {
    m_turret = turret;
    this.driveTrain = driveTrain;
    addRequirements(m_turret);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  public static double calculateAlignmentConvertedDegrees(Translation2d tagTranslation, Translation2d robotTranslation,
      Rotation2d robotRotation, double turretPosition) {
    Translation2d robotToTarget = tagTranslation
        .minus(robotTranslation); // gets x and y difference between robot and april tag
    Rotation2d turretAndRobot = robotRotation
        .plus(new Rotation2d(Math.toRadians(turretPosition)));// gets rotation of motor in relation to field
    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(turretAndRobot); // angle of x and y difference
                                                                                     // minue rotation between tag/robot
    return (180 - Math.abs(turretToTargetAngle.getDegrees()))
        * (turretToTargetAngle.getDegrees() / Math.abs(turretToTargetAngle.getDegrees())); // converts it into usable
                                                                                           // error for rotation
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // TODO: Fix turret alignment and angle measurment, currently jumping from ~-40
                          // to 40 or vice versa

    double convertedDeg = calculateAlignmentConvertedDegrees(m_tagpose.toPose2d().getTranslation(),
        m_StateMachine.getPose().getTranslation(), m_StateMachine.getPose().getRotation(),
        m_turret.getConvertedTurretPosition());

    if (driveTrain.getAngularVel().get() < 130) {
      double error = pid.calculate(m_turret.getConvertedTurretPosition(), convertedDeg); // sets turret speed
      m_turret.setSpeed(error);
    } else {
      m_turret.setSpeed(0);
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

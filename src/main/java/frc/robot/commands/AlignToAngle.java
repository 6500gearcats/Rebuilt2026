// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToAngle extends Command {
  /** Creates a new AlignToAngle. */
  private RobotStateMachine robotStateMachine = RobotStateMachine.getInstance();

  private Pose3d tagPose = Constants.APRIL_TAG_FIELD_LAYOUT.getTagPose(25).get();
  private CommandSwerveDrivetrain drivetrain;
  private SwerveRequest.FieldCentric drive;
 
  public AlignToAngle(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric drive) {
    this.drivetrain = drivetrain;
    this.drive = drive;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Aligning to tag");
    // This method will be called once per scheduler run
    double rectW = (tagPose.getX() + 1 - robotStateMachine.getPose().getX());
    double rectH = (tagPose.getY() + 1 - robotStateMachine.getPose().getY());
    
    SmartDashboard.putNumber("rectH", rectH);
    SmartDashboard.putNumber("rectW", rectW);
    

    double newAngle = Math.atan2(rectH, rectW) * (180 / Math.PI); // gets wanted angle for robot field
                                                                  // oriented

    SmartDashboard.putNumber("newAngle", newAngle);

    double newAngleRate = (newAngle - robotStateMachine.getPose().getRotation().getDegrees()) ;

    double kP = 0.3;

    double newNewAngleRate = newAngleRate * kP;
    // setPosition(newAngle);
    SmartDashboard.putNumber("newAngleRate", newAngleRate);

    CommandScheduler.getInstance().schedule(drivetrain.applyRequest(
                    () -> drive.withRotationalRate(newNewAngleRate)));
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    CommandScheduler.getInstance().cancel(drivetrain.getCurrentCommand());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.turret.Turret;

/** Aims at the cached shot target; zero yaw fires toward the robot's rear. */
public class AlignTurretToHub extends Command {
  private final Turret turret;
  private final RobotStateMachine stateMachine = RobotStateMachine.getInstance();

  public AlignTurretToHub(Turret turret) {
    this.turret = turret;
    addRequirements(turret);
  }

  @Override
  public void initialize() { SmartDashboard.putBoolean("Aligned", false); }

  @Override
  public void execute() {
    if (turret.isHoming()) {
      SmartDashboard.putBoolean("Aligned", false);
      return;
    }
    Translation2d toTarget = stateMachine.getTargetPose().getTranslation()
        .minus(stateMachine.getTurretPose().getTranslation());
    if (toTarget.getNorm() < 1e-9 || !turret.isHealthy()) {
      turret.setSpeed(0);
      SmartDashboard.putBoolean("Aligned", false);
      return;
    }
    double targetYawDegrees = MathUtil.inputModulus(toTarget.getAngle().getDegrees()
        - stateMachine.getPose().getRotation().getDegrees() - 180, -180, 180);
    // Keep the existing yaw travel limits. An unreachable target never passes alignment.
    turret.setPosition(MathUtil.clamp(targetYawDegrees, -103, 110));
    SmartDashboard.putBoolean("Aligned", stateMachine.isTurretAligned());
    SmartDashboard.putNumber("turning_pos_setpoint", targetYawDegrees);
  }

  @Override
  public void end(boolean interrupted) {
    turret.setSpeed(0);
    SmartDashboard.putBoolean("Aligned", false);
  }

  @Override
  public boolean isFinished() { return false; }
}

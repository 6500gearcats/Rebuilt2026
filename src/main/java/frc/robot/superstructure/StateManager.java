package frc.robot.superstructure;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotStateMachine;
import frc.robot.aiming.AimParams;

/**
 * Bridge between Hackbots Turret API and the Gearcats RobotStateMachine singleton.
 * Provides the pose, aim parameters, and shoot-ready signal that Turret.track() needs.
 */
public class StateManager {
  private final RobotStateMachine rsm;
  public final Trigger shootReady;

  public StateManager(RobotStateMachine rsm) {
    this.rsm = rsm;
    this.shootReady = new Trigger(rsm::isShootReady);
  }

  public Pose2d robotPose() {
    return rsm.getPose();
  }

  public AimParams aimParams() {
    return rsm.getAimParams();
  }
}

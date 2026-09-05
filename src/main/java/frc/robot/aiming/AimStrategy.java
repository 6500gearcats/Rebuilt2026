package frc.robot.aiming;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * An interface representing a method of aim-calculation; i.e. calculating what shot parameters are
 * necessary for a given robot configuration
 */
public interface AimStrategy {
  /**
   * Updates the AimParams with the new calculated shot based on the robot's state
   */
  public AimParams update(Pose3d target, Pose3d shooter, Translation2d velocity);
}

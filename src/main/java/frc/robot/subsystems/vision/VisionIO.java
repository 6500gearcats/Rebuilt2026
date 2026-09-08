package frc.robot.subsystems.vision;

import java.util.Optional;

/**
 * IO abstraction for a single vision camera source.
 *
 * <p>Implementations exist for real hardware ({@link frc.robot.subsystems.vision.photonvision.PhotonVisionIO}),
 * simulation ({@link frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO}), and
 * disabled-mode stubs. The {@link frc.robot.subsystems.vision.Vision} subsystem holds a
 * list of {@code VisionIO} instances and fuses their pose estimates into the Kalman filter.
 *
 * <p>All angle values are in degrees (PhotonVision convention). All range values are in meters.
 * When no target is visible, accessors return 0 as a safe default.
 */
public interface VisionIO {
    /** @return camera name for dashboards and logging. */
    public String getName();

    /** @return true if the camera should be used for pose estimation. */
    public boolean forPoseEstimation();

    /** @return yaw to the best target, in degrees. */
    public double getBestYaw();

    /** @return pitch to the best target, in degrees. */
    public double getBestPitch();

    /** @return range to the best target, in meters. */
    public double getBestRange();

    /** @return true if any targets are visible. */
    public boolean hasTargets();

    /**
     * Checks if the specified fiducial target is visible.
     *
     * @param fiducialID AprilTag or fiducial ID
     * @return true if the target is detected
     */
    public boolean hasChossenTarget(int fiducialID);

    /**
     * Gets the yaw to a chosen fiducial target.
     *
     * @param fiducialID AprilTag or fiducial ID
     * @return yaw in degrees
     */
    public double getChosenTargetYaw(int fiducialID);

    /**
     * Gets the pitch to a chosen fiducial target.
     *
     * @param fiducialID AprilTag or fiducial ID
     * @return pitch in degrees
     */
    public double getChosenTargetPitch(int fiducialID);

    /**
     * Gets the skew of a chosen fiducial target.
     *
     * @param fiducialID AprilTag or fiducial ID
     * @return skew angle in degrees
     */
    public double getChosenTargetSkew(int fiducialID);

    /**
     * Gets the range to a chosen fiducial target.
     *
     * @param fiducialID AprilTag or fiducial ID
     * @return range in meters
     */
    public double getChosenTargetRange(int fiducialID);

    /**
     * Returns a vision-based pose estimate if available.
     *
     * @return optional pose estimate
     */
    public Optional<VisionEstimate> getVisionEst();
}

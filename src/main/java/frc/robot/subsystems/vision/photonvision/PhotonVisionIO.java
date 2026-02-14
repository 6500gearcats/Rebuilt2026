package frc.robot.subsystems.vision.photonvision;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.subsystems.vision.VisionIO;

/**
 * VisionIO implementation backed by PhotonVision cameras.
 */
public class PhotonVisionIO implements VisionIO {
    private final PhotonCamera m_camera;
    public Translation3d robotToCameraTrl;
    public Rotation3d robotToCameraRot;
    public Transform3d robotToCamera;

    private boolean forPoseEstimation = true;

    private final PhotonPoseEstimator estimator;

    private double lastEstTimestamp = 0;
    public boolean isNewResult = false;

    /**
     * Creates a PhotonVision IO instance.
     *
     * @param cameraName        camera name as configured in PhotonVision
     * @param forPoseEstimation true to use for pose estimation
     * @param robotToCameraTrl  translation from robot to camera
     * @param robotToCameraRot  rotation from robot to camera
     */
    public PhotonVisionIO(String cameraName, boolean forPoseEstimation, Translation3d robotToCameraTrl,
            Rotation3d robotToCameraRot) {
        this.forPoseEstimation = forPoseEstimation;
        this.robotToCameraTrl = robotToCameraTrl;
        this.robotToCameraRot = robotToCameraRot;
        this.robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
        m_camera = new PhotonCamera(cameraName);

        estimator = new PhotonPoseEstimator(
                kTagLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCamera);
    }

    @Override
    public String getName() {
        return m_camera.getName();
    }

    @Override
    public boolean forPoseEstimation() {
        return forPoseEstimation;
    }

    @Override
    public double getBestYaw() {
        var result = getLatestResult();
        double yaw = 0.0;
        if (result.hasTargets()) {
            // Calculate angular turn power
            // Remove -1.0 because it was inverting results.
            yaw = result.getBestTarget().getYaw();
        }
        return yaw;
    }

    public PhotonPipelineResult getLatestResult() {
        return m_camera.getLatestResult();
    }

    @Override
    public double getBestPitch() {
        var result = getLatestResult();
        double pitch = 0.0;
        if (result.hasTargets()) {
            // Calculate angular turn power
            // Remove -1.0 because it was inverting results.
            pitch = result.getBestTarget().getPitch();
        }

        return pitch;
    }

    @Override
    public double getBestRange() {
        var result = getLatestResult();
        double range = 0;
        if (result.hasTargets()) {
            range = PhotonUtils.calculateDistanceToTargetMeters(
                    0,
                    0,
                    0,
                    Units.degreesToRadians(result.getBestTarget().getPitch()));
        }
        return range;
    }

    @Override
    public double getChosenTargetYaw(int fiducialID) {
        var result = getLatestResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == fiducialID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getYaw();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    @Override
    public double getChosenTargetPitch(int fiducialID) {
        var result = getLatestResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == fiducialID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getPitch();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    @Override
    public double getChosenTargetSkew(int fiducialID) {
        var result = getLatestResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();
        double rotation = 0;

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (result.hasTargets()) {
                if (target.getFiducialId() == fiducialID) {
                    // Use the value of target to find our rotation using the getYaw command
                    return target.getSkew();
                }
            } else {
                rotation = 0;
            }
        }

        return rotation;
    }

    @Override
    public double getChosenTargetRange(int fiducialID) {
        var result = getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        double range = 0;
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == fiducialID) {
                    range = PhotonUtils.calculateDistanceToTargetMeters(
                            0,
                            0,
                            0,
                            Units.degreesToRadians(target.getPitch()));
                    return range;
                }
            }
        }
        return 0;
    }

    @Override
    public boolean hasTargets() {
        var result = getLatestResult();
        if (result.hasTargets()) {
            return true;
        }
        return false;
    }

    @Override
    public boolean hasChossenTarget(int fiducialID) {
        var result = getLatestResult();
        // Get a list of all of the targets that have been detected.
        List<PhotonTrackedTarget> targets = result.getTargets();

        // For each target we have check if it matches the id you want.
        for (PhotonTrackedTarget target : targets) {
            if (target.getFiducialId() == fiducialID) {
                // Use the value of target to find our rotation using the getYaw command
                return true;
            }
        }

        return false;
    }

    /**
     * The standard deviations of the estimated pose from
     * {@link #getEstimatedGlobalPose()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty())
                continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0)
            return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
            estStdDevs = VisionConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
            estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        return estStdDevs;
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@link Optional<Pose2D>>} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<VisionEstimate> getVisionEst() {
        PhotonPipelineResult result = getLatestResult();
        var visionEst = estimator.update(result);
        double latestTimestamp = result.getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (newResult) {
            lastEstTimestamp = latestTimestamp;
            isNewResult = true;
        } else {
            isNewResult = false;
        }

        if (visionEst.isEmpty()) {
            return Optional.empty();
        }
        return Optional.of(new VisionEstimate(visionEst.get()));
    }
}

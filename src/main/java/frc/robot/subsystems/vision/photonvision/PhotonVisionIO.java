package frc.robot.subsystems.vision.photonvision;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.DashboardTelemetry;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.subsystems.vision.VisionIO;

/**
 * VisionIO implementation backed by PhotonVision cameras.
 */
public class PhotonVisionIO implements VisionIO {
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
    private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 6.0;
    private static final double MAX_POSE_Z_METERS = 1.00;

    private final PhotonCamera m_camera;
    private final PhotonResultBuffer results;
    private final DashboardTelemetry.DebugPublisher debugTelemetry = new DashboardTelemetry.DebugPublisher();
    private Pose2d latestAcceptedPose;
    private Boolean latestPoseRejected;
    public Translation3d robotToCameraTrl;
    public Rotation3d robotToCameraRot;
    public Transform3d robotToCamera;

    private final boolean forPoseEstimation;

    private final PhotonPoseEstimator estimator;
    public Field2d m_field = new Field2d();

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
        results = new PhotonResultBuffer(m_camera::getAllUnreadResults);
        estimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);

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

    @Override
    public void updateInputs() {
        results.update();
        double timestamp = results.latest().getTimestampSeconds();
        isNewResult = Math.abs(timestamp - lastEstTimestamp) > 1e-5;
        if (isNewResult) {
            lastEstTimestamp = timestamp;
        }
    }

    /** Returns the snapshot captured by updateInputs; never drains the camera queue. */
    public PhotonPipelineResult getLatestResult() {
        return results.latest();
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
        if (result.hasTargets()) {
            for (PhotonTrackedTarget target : targets) {
                if (target.getFiducialId() == fiducialID) {
                    return PhotonUtils.calculateDistanceToTargetMeters(
                            0,
                            0,
                            0,
                            Units.degreesToRadians(target.getPitch()));
                }
            }
        }
        return 0;
    }

    @Override
    public boolean hasTargets() {
        return getLatestResult().hasTargets();
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
     * The standard deviations of an estimated pose from
     * {@link #getVisionEstimates()}, for use
     * with {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}.
     * This should only be used when there are targets visible.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(
            Pose2d estimatedPose,
            List<PhotonTrackedTarget> targets) {
        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        int numTags = 0;
        double avgDist = 0.0;

        for (var tgt : targets) {
            var tagPose = kTagLayout.getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;

            numTags++;
            avgDist += tagPose.get().toPose2d()
                    .getTranslation()
                    .getDistance(estimatedPose.getTranslation());
        }

        if (numTags == 0) return estStdDevs;

        avgDist /= numTags;
        if (numTags > 1) {
            estStdDevs = VisionConstants.kMultiTagStdDevs;
        } else if (avgDist > 4.0) {
            return VecBuilder.fill(
                    Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        return estStdDevs.times(1.0 + avgDist * avgDist / 30.0);
    }

    /**
     * Rejects estimates that are not trustworthy enough to enter pose fusion.
     * Ambiguity is evaluated only for single-tag estimates; multi-tag estimates
     * are screened using geometry and the number of tags used.
     */
    private boolean shouldRejectEstimate(EstimatedRobotPose estimate) {
        if (estimate.targetsUsed.isEmpty()) {
            return true;
        }

        Pose3d pose = estimate.estimatedPose;
        if (!Double.isFinite(pose.getX())
                || !Double.isFinite(pose.getY())
                || !Double.isFinite(pose.getZ())
                || Math.abs(pose.getZ()) > MAX_POSE_Z_METERS) {
            return true;
        }

        if (estimate.targetsUsed.size() == 1) {
            PhotonTrackedTarget target = estimate.targetsUsed.get(0);
            double ambiguity = target.getPoseAmbiguity();
            double distance = target.getBestCameraToTarget()
                    .getTranslation()
                    .getNorm();

            if (!Double.isFinite(ambiguity)
                    || ambiguity < 0.0
                    || ambiguity > MAX_SINGLE_TAG_AMBIGUITY) {
                return true;
            }

            return !Double.isFinite(distance)
                    || distance > MAX_SINGLE_TAG_DISTANCE_METERS;
        }

        return false;
    }

    /**
     * Prefer the coprocessor multi-tag solution, but allow a single-tag
     * lowest-ambiguity solution when no multi-tag result is available.
     */
    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> multiTagEstimate =
                estimator.estimateCoprocMultiTagPose(result);
        if (multiTagEstimate.isPresent()) {
            return multiTagEstimate;
        }

        return estimator.estimateLowestAmbiguityPose(result);
    }

    /**
     * The latest estimated robot pose on the field from vision data. This may be
     * empty. This should
     * only be called once per loop.
     *
     * @return An {@code Optional<VisionEstimate>} with an estimated pose, estimate
     *         timestamp, and targets
     *         used for estimation.
     */
    public Optional<VisionEstimate> getVisionEst() {
        Optional<VisionEstimate> latestEstimate = Optional.empty();

        // Consume this loop's batch and keep the newest valid estimate.
        for (PhotonPipelineResult result : results.takeUnread()) {
            Optional<EstimatedRobotPose> estimatedPose =
                    estimatePose(result);

            if (estimatedPose.isPresent()) {
                EstimatedRobotPose estimate = estimatedPose.get();
                if (shouldRejectEstimate(estimate)) {
                    latestPoseRejected = true;
                    continue;
                }
                latestPoseRejected = false;

                Matrix<N3, N1> stdDevs =
                        getEstimationStdDevs(
                                estimate.estimatedPose.toPose2d(),
                                estimate.targetsUsed);
                latestEstimate = Optional.of(new VisionEstimate(estimate, stdDevs));
            }
        }

        latestEstimate.ifPresent(est -> publishCameraPose(est.getPose()));

        publishDebugTelemetry();
        return latestEstimate;
    }

    @Override
    public List<VisionEstimate> getVisionEstimates() {
        List<VisionEstimate> estimates = new ArrayList<>();

        // updateInputs drains the camera once per loop; consume the cached batch once.
        for (PhotonPipelineResult result : results.takeUnread()) {
            Optional<EstimatedRobotPose> estimatedPose =
                    estimatePose(result);

            estimatedPose.ifPresent(estimate -> {
                if (shouldRejectEstimate(estimate)) {
                    latestPoseRejected = true;
                    return;
                }
                latestPoseRejected = false;

                Matrix<N3, N1> stdDevs =
                        getEstimationStdDevs(
                                estimate.estimatedPose.toPose2d(),
                                estimate.targetsUsed);
                VisionEstimate est = new VisionEstimate(estimate, stdDevs);
                estimates.add(est);
            });
        }

        if (!estimates.isEmpty()) {
            publishCameraPose(estimates.get(estimates.size() - 1).getPose());
        }

        publishDebugTelemetry();
        return estimates;
    }

    private void publishCameraPose(Pose2d pose) {
        latestAcceptedPose = pose;
    }

    private void publishDebugTelemetry() {
        if (!debugTelemetry.shouldPublish()) {
            return;
        }
        if (latestAcceptedPose != null) {
            debugTelemetry.putData("Debug/Vision/CamPose/" + m_camera.getName(), m_field);
            m_field.setRobotPose(latestAcceptedPose);
        }
        if (latestPoseRejected != null) {
            SmartDashboard.putBoolean("Debug/Vision/Pose Rejected/" + m_camera.getName(), latestPoseRejected);
        }
    }
}

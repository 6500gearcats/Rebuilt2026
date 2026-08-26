package frc.robot.subsystems.vision.photonvision;

import static frc.robot.Constants.VisionConstants.kTagLayout;

import java.io.BufferedWriter;
import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.subsystems.vision.VisionIO;

/**
 * VisionIO implementation backed by PhotonVision cameras.
 */
public class PhotonVisionIO implements VisionIO {
    private static final String COLLECTION_ROOT = "Vision/DataCollection/";
    private static final DateTimeFormatter FILE_STAMP = DateTimeFormatter.ofPattern("yyyyMMdd-HHmmss");
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20;
    private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0;
    private static final double MAX_POSE_Z_METERS = 1.00;

    private final PhotonCamera m_camera;
    public Translation3d robotToCameraTrl;
    public Rotation3d robotToCameraRot;
    public Transform3d robotToCamera;

    private boolean forPoseEstimation = true;

    private final PhotonPoseEstimator estimator;
    private final VisionTrialLogger trialLogger = new VisionTrialLogger();
    private VisionCamera m_visionCam;
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
        SmartDashboard.putData("CamPose" + m_camera.getName(), m_field);
        trialLogger.publishDashboardDefaults();
        estimator = new PhotonPoseEstimator(kTagLayout, robotToCamera);

        m_visionCam = new VisionCamera(cameraName,m_camera,estimator);
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

            if (!Double.isFinite(distance)
                    || distance > MAX_SINGLE_TAG_DISTANCE_METERS) {
                return true;
            }
        }

        return false;
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
        Optional<VisionEstimate> latestEstimate = Optional.empty();

        // Drain PhotonVision's FIFO and keep the newest valid estimate.
        for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimatedPose =
                    estimator.estimateCoprocMultiTagPose(result);
            m_visionCam.latestResult = Optional.of(result);
            m_visionCam.latestPose = estimatedPose;
            trialLogger.log(m_visionCam, result, estimatedPose);

            double latestTimestamp = result.getTimestampSeconds();
            isNewResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
            if (isNewResult) {
                lastEstTimestamp = latestTimestamp;
            }

            if (estimatedPose.isPresent()) {
                EstimatedRobotPose estimate = estimatedPose.get();
                if (shouldRejectEstimate(estimate)) {
                    continue;
                }

                Matrix<N3, N1> stdDevs =
                        getEstimationStdDevs(
                                estimate.estimatedPose.toPose2d(),
                                estimate.targetsUsed);
                latestEstimate = Optional.of(new VisionEstimate(estimate, stdDevs));
            }
        }

        latestEstimate.ifPresent(est -> publishCameraPose(est.getPose()));

        return latestEstimate;
    }

    @Override
    public List<VisionEstimate> getVisionEstimates() {
        List<VisionEstimate> estimates = new ArrayList<>();

        // Call exactly once per robot loop: this drains PhotonVision's FIFO.
        for (PhotonPipelineResult result : m_camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimatedPose =
                    estimator.estimateCoprocMultiTagPose(result);
            m_visionCam.latestResult = Optional.of(result);
            m_visionCam.latestPose = estimatedPose;
            //trialLogger.log(m_visionCam, result, estimatedPose);

            estimatedPose.ifPresent(estimate -> {
                if (shouldRejectEstimate(estimate)) {
                    SmartDashboard.putBoolean("CamPoseRejected" + m_camera.getName(),true);
                    return;
                }
                SmartDashboard.putBoolean("CamPoseRejected" + m_camera.getName(),false);

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

        return estimates;
    }

    private void publishCameraPose(Pose2d pose) {
        m_field.setRobotPose(pose);
        SmartDashboard.putData("CamPose" + m_camera.getName(), m_field);
    }

    private static final class VisionCamera {
        private final String name;
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;
        private Optional<PhotonPipelineResult> latestResult = Optional.empty();
        private Optional<EstimatedRobotPose> latestPose = Optional.empty();

        private VisionCamera(String name, PhotonCamera camera, PhotonPoseEstimator poseEstimator) {
            this.name = name;
            this.camera = camera;
            this.poseEstimator = poseEstimator;
        }
    }

    private static final class VisionTrialLogger {
        private static final String CSV_HEADER = String.join(
            ",",
            "trial_id",
            "trial_label",
            "trial_notes",
            "trial_elapsed_sec",
            "result_timestamp_sec",
            "camera_name",
            "sequence_id",
            "has_targets",
            "visible_tag_count",
            "estimated_pose_present",
            "estimated_x_m",
            "estimated_y_m",
            "estimated_z_m",
            "estimated_rotation_deg",
            "expected_x_m",
            "expected_y_m",
            "expected_rotation_deg",
            "x_error_m",
            "y_error_m",
            "rotation_error_deg",
            "strategy",
            "targets_used_count",
            "targets_used_ids",
            "best_target_id",
            "best_target_yaw_deg",
            "best_target_pitch_deg",
            "best_target_area_pct",
            "best_target_skew_deg",
            "best_target_ambiguity",
            "best_target_distance_m",
            "avg_tag_distance_m",
            "min_tag_distance_m",
            "max_tag_distance_m",
            "multitag_present"
        );

        private boolean loggingActive;
        private double trialStartTimestampSec;
        private String trialId = "";
        private Path activeLogPath;
        private BufferedWriter writer;

        void publishDashboardDefaults() {
            SmartDashboard.putString(COLLECTION_ROOT + "Trial Label", "grid-A1-0deg");
            SmartDashboard.putString(COLLECTION_ROOT + "Trial Notes", "");
            SmartDashboard.putNumber(COLLECTION_ROOT + "Expected X (m)", 0.0);
            SmartDashboard.putNumber(COLLECTION_ROOT + "Expected Y (m)", 0.0);
            SmartDashboard.putNumber(COLLECTION_ROOT + "Expected Heading (deg)", 0.0);
            SmartDashboard.putNumber(COLLECTION_ROOT + "Capture Duration (s)", 10.0);
            SmartDashboard.putBoolean(COLLECTION_ROOT + "Start Logging", false);
            SmartDashboard.putBoolean(COLLECTION_ROOT + "Stop Logging", false);
            SmartDashboard.putBoolean(COLLECTION_ROOT + "Logging Active", false);
            SmartDashboard.putNumber(COLLECTION_ROOT + "Elapsed (s)", 0.0);
            SmartDashboard.putString(COLLECTION_ROOT + "Current Trial Id", "");
            SmartDashboard.putString(COLLECTION_ROOT + "Current Log File", "");
            SmartDashboard.putString(COLLECTION_ROOT + "Status", "Idle");
        }

        void updateControlState() {
            if (SmartDashboard.getBoolean(COLLECTION_ROOT + "Start Logging", false)) {
                SmartDashboard.putBoolean(COLLECTION_ROOT + "Start Logging", false);
                startTrial();
            }

            if (SmartDashboard.getBoolean(COLLECTION_ROOT + "Stop Logging", false)) {
                SmartDashboard.putBoolean(COLLECTION_ROOT + "Stop Logging", false);
                stopTrial("Stopped from dashboard");
            }

            if (loggingActive) {
                double elapsed = nowSeconds() - trialStartTimestampSec;
                SmartDashboard.putNumber(COLLECTION_ROOT + "Elapsed (s)", elapsed);

                double durationSeconds = SmartDashboard.getNumber(COLLECTION_ROOT + "Capture Duration (s)", 10.0);
                if (durationSeconds > 0.0 && elapsed >= durationSeconds) {
                    stopTrial("Capture duration reached");
                }
            }
        }

        void log(
                VisionCamera camera,
                PhotonPipelineResult result,
                Optional<EstimatedRobotPose> estimatedPose) {
            if (!loggingActive || writer == null) {
                return;
            }

        PhotonTrackedTarget bestTarget = result.hasTargets() ? result.getBestTarget() : null;
        Pose2d pose2d = estimatedPose.map(pose -> pose.estimatedPose.toPose2d()).orElse(null);

        double expectedX = SmartDashboard.getNumber(COLLECTION_ROOT + "Expected X (m)", 0.0);
        double expectedY = SmartDashboard.getNumber(COLLECTION_ROOT + "Expected Y (m)", 0.0);
        double expectedHeadingDeg = SmartDashboard.getNumber(COLLECTION_ROOT + "Expected Heading (deg)", 0.0);
        double estimatedX = pose2d != null ? pose2d.getX() : Double.NaN;
        double estimatedY = pose2d != null ? pose2d.getY() : Double.NaN;
        double estimatedZ = estimatedPose.map(pose -> pose.estimatedPose.getZ()).orElse(Double.NaN);
        double estimatedHeadingDeg = pose2d != null ? pose2d.getRotation().getDegrees() : Double.NaN;

            List<PhotonTrackedTarget> targetsForDistance = estimatedPose.map(pose -> pose.targetsUsed).orElse(result.getTargets());
            List<Double> distances = targetsForDistance.stream()
                .map(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .toList();

            double avgDistance = distances.isEmpty()
                ? Double.NaN
                : distances.stream().mapToDouble(Double::doubleValue).average().orElse(Double.NaN);
            double minDistance = distances.isEmpty()
                ? Double.NaN
                : distances.stream().mapToDouble(Double::doubleValue).min().orElse(Double.NaN);
            double maxDistance = distances.isEmpty()
                ? Double.NaN
                : distances.stream().mapToDouble(Double::doubleValue).max().orElse(Double.NaN);

            List<Integer> targetsUsedIds = estimatedPose
                .map(pose -> pose.targetsUsed.stream().map(PhotonTrackedTarget::getFiducialId).toList())
                .orElseGet(ArrayList::new);

            try {
                writer.write(csvRow(
                    trialId,
                    SmartDashboard.getString(COLLECTION_ROOT + "Trial Label", ""),
                    SmartDashboard.getString(COLLECTION_ROOT + "Trial Notes", ""),
                    nowSeconds() - trialStartTimestampSec,
                    result.getTimestampSeconds(),
                    camera.name,
                    result.metadata.sequenceID,
                    result.hasTargets(),
                    result.getTargets().size(),
                    estimatedPose.isPresent(),
                    estimatedX,
                    estimatedY,
                    estimatedZ,
                    estimatedHeadingDeg,
                    expectedX,
                    expectedY,
                    expectedHeadingDeg,
                    estimatedPose.isPresent() ? estimatedX - expectedX : Double.NaN,
                    estimatedPose.isPresent() ? estimatedY - expectedY : Double.NaN,
                    estimatedPose.isPresent() ? wrapDegrees(estimatedHeadingDeg - expectedHeadingDeg) : Double.NaN,
                    estimatedPose.map(pose -> String.valueOf(pose.strategy)).orElse(""),
                    targetsForDistance.size(),
                    joinIds(targetsUsedIds),
                    bestTarget != null ? bestTarget.getFiducialId() : -1,
                    bestTarget != null ? bestTarget.getYaw() : Double.NaN,
                    bestTarget != null ? bestTarget.getPitch() : Double.NaN,
                    bestTarget != null ? bestTarget.getArea() : Double.NaN,
                    bestTarget != null ? bestTarget.getSkew() : Double.NaN,
                    bestTarget != null ? bestTarget.getPoseAmbiguity() : Double.NaN,
                    bestTarget != null ? bestTarget.getBestCameraToTarget().getTranslation().getNorm() : Double.NaN,
                    avgDistance,
                    minDistance,
                    maxDistance,
                    result.getMultiTagResult().isPresent()
                ));
                writer.newLine();
                writer.flush();
            } catch (IOException ex) {
                throw new UncheckedIOException("Failed to write vision trial log to " + activeLogPath, ex);
            }
        }

        private void startTrial() {
            stopTrial("Restarting logger");

            trialId = FILE_STAMP.format(LocalDateTime.now());
            trialStartTimestampSec = nowSeconds();
            activeLogPath = createLogPath(trialId);

            try {
                Files.createDirectories(activeLogPath.getParent());
                writer = Files.newBufferedWriter(activeLogPath);
                writer.write(CSV_HEADER);
                writer.newLine();
                writer.flush();
            } catch (IOException ex) {
                throw new UncheckedIOException("Failed to create vision trial log at " + activeLogPath, ex);
            }

            loggingActive = true;
            SmartDashboard.putBoolean(COLLECTION_ROOT + "Logging Active", true);
            SmartDashboard.putNumber(COLLECTION_ROOT + "Elapsed (s)", 0.0);
            SmartDashboard.putString(COLLECTION_ROOT + "Current Trial Id", trialId);
            SmartDashboard.putString(COLLECTION_ROOT + "Current Log File", activeLogPath.toString());
            SmartDashboard.putString(COLLECTION_ROOT + "Status", "Logging");
        }

        private void stopTrial(String reason) {
            if (writer != null) {
                try {
                    writer.flush();
                    writer.close();
                } catch (IOException ex) {
                    throw new UncheckedIOException("Failed to close vision trial log at " + activeLogPath, ex);
                } finally {
                    writer = null;
                }
            }

            loggingActive = false;
            SmartDashboard.putBoolean(COLLECTION_ROOT + "Logging Active", false);
            SmartDashboard.putString(COLLECTION_ROOT + "Status", reason);
        }

        private static Path createLogPath(String trialId) {
            return Filesystem.getOperatingDirectory().toPath()
                .resolve("vision-data")
                .resolve("photonmq-" + trialId + ".csv");
        }

        private static String joinIds(List<Integer> ids) {
            return ids.stream().map(String::valueOf).collect(Collectors.joining("|"));
        }

        private static double nowSeconds() {
            return System.currentTimeMillis() / 1000.0;
        }

        private static double wrapDegrees(double degrees) {
            double wrapped = degrees % 360.0;
            if (wrapped > 180.0) {
                wrapped -= 360.0;
            } else if (wrapped < -180.0) {
                wrapped += 360.0;
            }
            return wrapped;
        }

        private static String csvRow(Object... values) {
            return java.util.Arrays.stream(values)
                .map(VisionTrialLogger::csvCell)
                .collect(Collectors.joining(","));
        }

        private static String csvCell(Object value) {
            String text = String.valueOf(value == null ? "" : value);
            String escaped = text.replace("\"", "\"\"");
            return "\"" + escaped + "\"";
        }
    }
}

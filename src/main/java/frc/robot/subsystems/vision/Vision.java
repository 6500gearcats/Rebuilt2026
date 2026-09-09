// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO;

/**
 * Fuses AprilTag pose estimates from one or more cameras into a
 * {@link SwerveDrivePoseEstimator} (Kalman filter) to produce a single best-estimate
 * robot pose for the aiming and autonomous pipelines.
 *
 * <h2>How the Kalman Filter Works (brief)</h2>
 * The filter maintains a probability distribution over the robot's pose. It has two
 * sources of information:
 * <ul>
 *   <li><b>Odometry (wheel encoders + gyro):</b> accurate over short distances but drifts
 *       over time. Tuned with {@code m_stateStndDev} — smaller numbers = trust odometry more.
 *   <li><b>Vision (AprilTag detections):</b> accurate globally but noisy measurement-to-
 *       measurement. Tuned with the dynamic {@code stdDevs} in {@link #periodic()} — smaller
 *       numbers = trust vision more.
 * </ul>
 * The filter blends both sources optimally, weighting each by its stated uncertainty.
 *
 * <h2>Multi-Camera Fusion</h2>
 * Any number of {@link VisionIO} instances can be passed to the full constructor. Only those
 * that return {@code true} from {@link VisionIO#forPoseEstimation()} are used for Kalman
 * filter updates; others may be used for non-localization tasks (e.g., range finding).
 *
 * <h2>Replay Mode</h2>
 * Constructing with the no-arg constructor sets {@code isReplay = true}, which bypasses all
 * estimation logic. This is a placeholder for AdvantageKit log-replay — the pose would instead
 * come from replayed log inputs rather than live cameras.
 */
public class Vision extends SubsystemBase {
  /**
   * Odometry trust matrix [x, y, θ] in meters/radians.
   * Current values: trust odometry tightly (0.1 m, 0.1 m, 0.1°). Increase to trust
   * wheel encoders less when slippage or loop overruns cause odometry drift.
   */
  private static final Vector<N3> m_stateStndDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));

  /**
   * Base vision trust matrix [x, y, θ] in meters/radians — used as the starting point
   * before the distance-scaled penalty in {@link #periodic()} is applied.
   * Increase to trust AprilTag detections less globally.
   */
  private static final Vector<N3> m_visionStndDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));

  @SuppressWarnings("unused")
  private final VisionIO[] io;
  private AprilTagFieldLayout tagLayout;

  private VisionSystemSim sim = null;
  private final ArrayList<VisionIO> m_visionOdometryCams = new ArrayList<VisionIO>();
  private final ArrayList<PhotonVisionSimIO> m_simCameras = new ArrayList<PhotonVisionSimIO>();
  private final ArrayList<PhotonVisionSimIO> m_turretCamSims = new ArrayList<PhotonVisionSimIO>();

  SwerveDrivePoseEstimator estimator;

  private Supplier<Rotation2d> m_rotationSupplier;
  private Supplier<SwerveModulePosition[]> m_swerveModulePositionSupplier;
  private Supplier<Pose2d> m_poseSupplier;

  private boolean isReplay = false;
  private int m_loopCount = 0;

  public Field2d m_field = new Field2d();

  private final StructPublisher<Pose2d> gccPub = NetworkTableInstance.getDefault()
      .getTable("StateMachine")
      .getStructTopic("GCC", Pose2d.struct)
      .publish();

  private final StructPublisher<Pose2d> gcdPub = NetworkTableInstance.getDefault()
      .getTable("StateMachine")
      .getStructTopic("GCD", Pose2d.struct)
      .publish();

  /**
   * Creates a vision subsystem with live camera IO.
   *
   * @param rotationSupplier             drivetrain rotation supplier
   * @param swerveModulePositionSupplier drivetrain module positions
   * @param poseSupplier                 drivetrain pose supplier
   * @param io                           vision IO instances
   */
  public Vision(Supplier<Rotation2d> rotationSupplier,
      Supplier<SwerveModulePosition[]> swerveModulePositionSupplier, Supplier<Pose2d> poseSupplier, VisionIO... io) {

    this.io = io;
    this.m_rotationSupplier = rotationSupplier;
    this.m_swerveModulePositionSupplier = swerveModulePositionSupplier;
    this.m_poseSupplier = poseSupplier;
    estimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        m_rotationSupplier.get(),
        m_swerveModulePositionSupplier.get(),
        new Pose2d(),
        m_stateStndDev,
        m_visionStndDev);

    for (VisionIO visionIO : io) {
      if (visionIO.forPoseEstimation()) {
        m_visionOdometryCams.add(visionIO);
      }
      if (visionIO instanceof PhotonVisionSimIO) {
        if (sim == null) {
          sim = new VisionSystemSim("main");
        }
        PhotonVisionSimIO cameraSim = (PhotonVisionSimIO) visionIO;
        if (cameraSim.isMountedOnTurret()) {
          m_turretCamSims.add(cameraSim);
        }
        m_simCameras.add(cameraSim);
      }
    }
    if (sim != null) {
      setUpSim();
    }
    SmartDashboard.putData("Field", m_field);
  }

  /**
   * No-arg constructor — activates replay mode. All estimation is bypassed;
   * {@link #periodic()} and {@link #simulationPeriodic()} return immediately.
   * Used as a placeholder when AdvantageKit log-replay is implemented.
   */
  public Vision() {
    io = null;
    estimator = null;
    isReplay = true;
  }

  /**
   * Runs every 20 ms. Performs three stages:
   *
   * <ol>
   *   <li><b>Odometry propagation:</b> updates the Kalman filter with the latest wheel encoder
   *       positions and gyro heading. This step runs even when no vision targets are visible and
   *       keeps the pose estimate moving correctly between vision updates.
   *   <li><b>Vision fusion:</b> for each camera flagged for pose estimation, calls
   *       {@link VisionIO#getVisionEst()} once per loop.
   *       <ul>
   *         <li>Measurements more than 4 m from the current odometry estimate are rejected
   *             outright — this guards against tag-ID misdetections or extreme lens distortion
   *             that would otherwise jump the pose wildly.
   *         <li>Accepted measurements are weighted by a distance-scaled standard deviation:
   *             {@code σ = 0.1 + dist * 0.05} for x and y (meters), and
   *             {@code σ = 10° + dist * 5°} for heading. Farther detections are trusted less
   *             because pixel errors project to larger field errors at range.
   *       </ul>
   *   <li><b>Field2d telemetry:</b> in simulation the field widget shows the ground-truth pose
   *       from the drivetrain; on hardware it shows the filter's own estimate.
   * </ol>
   */
  @Override
  public void periodic() {
    if (isReplay) {
      return;
    }

    // In simulation every getState() call on the CTRE drivetrain contends with the
    // 5ms Phoenix 6 sim notifier write-lock, stacking into 70ms+ overruns. Skip the
    // entire periodic body in sim — the robot is visible in AdvantageScope via the
    // DriveState/Pose struct that Telemetry publishes independently of this subsystem.
    if (RobotBase.isSimulation()) return;

    m_loopCount++;

    // Rate-limit to every 4th loop (~12.5 Hz). Camera frames arrive at 30-60 FPS max;
    // running pose estimation at 50 Hz wastes loop budget without improving accuracy.
    if (m_loopCount % 4 != 0) {
      return;
    }

    estimator.update(
        m_rotationSupplier.get(),
        m_swerveModulePositionSupplier.get());

    for (VisionIO visionIO : m_visionOdometryCams) {
      Optional<VisionEstimate> est = visionIO.getVisionEst();

      est.ifPresent(e -> {
        Pose2d currentEst = estimator.getEstimatedPosition();
        double dist = currentEst.getTranslation().getDistance(e.getPose().getTranslation());

        // Reject detections more than 4 m from current odometry — guards against
        // bad tag-ID detections or extreme lens distortion.
        if (dist > 4.0) return;

        // Scale standard deviation by distance: farther detections are less precise.
        Matrix<N3, N1> stdDevs = VecBuilder.fill(
            0.1 + dist * 0.05,
            0.1 + dist * 0.05,
            Units.degreesToRadians(10 + dist * 5));
        estimator.addVisionMeasurement(e.getPose(), e.getTimestamp(), stdDevs);
      });

      if (visionIO.getName().contains("gcc")) {
        est.ifPresent(e -> gccPub.set(e.getPose()));
      } else if (visionIO.getName().contains("gcd")) {
        est.ifPresent(e -> gcdPub.set(e.getPose()));
      }
    }

    m_field.setRobotPose(estimator.getEstimatedPosition());
  }

  /**
   * Runs every 20 ms in simulation only. Advances the PhotonVision simulation by one step,
   * using the drivetrain's ground-truth pose from {@code m_poseSupplier}.
   *
   * <p>For cameras mounted on the turret, the camera transform is updated by the current turret
   * rotation before the simulation frame is computed. The rotation is currently hardcoded to 5°
   * as a placeholder — TODO: replace with the actual turret angle from the {@link
   * frc.robot.subsystems.turret.Turret} subsystem once the turret is wired in (Stage 8).
   */
  @Override
  public void simulationPeriodic() {
    if (isReplay) {
      return;
    }
    if (m_turretCamSims.size() > 0) {
      for (PhotonVisionSimIO cameraSim : m_turretCamSims) {
        Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(5));
        Transform3d robotToCamera = new Transform3d(
            cameraSim.robotToCameraTrl.rotateBy(turretRotation),
            cameraSim.robotToCameraRot.rotateBy(turretRotation));
        sim.adjustCamera(cameraSim.getCameraSim(), robotToCamera);
      }
    }
    sim.update(m_poseSupplier.get());
  }

  /**
   * Registers the field AprilTag layout and all cameras with the PhotonVision
   * simulation system. Called once from the full constructor when at least one
   * {@link PhotonVisionSimIO} is detected in the IO list.
   */
  public void setUpSim() {
    tagLayout = VisionConstants.kTagLayout;
    sim.addAprilTags(tagLayout);
    for (PhotonVisionSimIO cameraSim : m_simCameras) {
      sim.addCamera(cameraSim.getCameraSim(), cameraSim.robotToCamera);
    }
  }

  /**
   * Returns the Kalman filter's current best-estimate robot pose.
   * May be stale by up to one loop cycle (20 ms).
   */
  public Pose2d getEstimatedPose() {
    return estimator.getEstimatedPosition();
  }

  /**
   * Resets the Kalman filter's internal pose to {@code pose}.
   * Use at auto start or after a known field position is established.
   */
  public void resetVisionPose(Pose2d pose) {
    estimator.resetPose(pose);
  }
}
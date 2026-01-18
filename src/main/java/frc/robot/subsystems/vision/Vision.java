// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.function.Supplier;

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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.vision.photonvision.PhotonVisionSimIO;

public class Vision extends SubsystemBase {
  /*
   * This uses the Kulman Filter to estimate the pose of the robot.
   * HIGHLY RECOMMENDED to research the Kalman Filter to properly understand this.
   */

  /*
   * You can tune Standard deviation of repective estimation/measurement to
   * configure how much you trust them.
   * Smaller numbers will cause the filter to
   * "trust" the estimate from that particular component more than the others.
   * This in turn means the particualr component will have a stronger influence
   * on the final pose estimate.
   */

  /*
   * The Standard Deviation for the Model's States.
   * Increase numbers to trust the model's state estimates less
   * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians,
   * then meters.
   */
  private static final Vector<N3> m_stateStndDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));

  /*
   * The Standard Deviation for the Vsion Measurements (i,e. NOSIE);
   * Increase numbers to trust mesurements from Vision less.
   * This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
   */
  private static final Vector<N3> m_visionStndDev = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(0.1));

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

  public Field2d m_field = new Field2d();

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
        if (visionIO instanceof PhotonVisionSimIO) {
            if (sim == null) {
                sim = new VisionSystemSim("main");
            }
            PhotonVisionSimIO cameraSim = (PhotonVisionSimIO) visionIO;
            if(cameraSim.isMountedOnTurret()) {
              m_turretCamSims.add(cameraSim);
            }
            m_simCameras.add(cameraSim);
        }
    }
    if(sim != null) {
        setUpSim();
    }
    SmartDashboard.putData("Field", m_field);
  }

  public Vision() {
    io = null;
    estimator = null;
    isReplay = true;
  }

  @Override
  public void periodic() {
    if(isReplay) {
      return;
    }
    estimator.updateWithTime(
        System.currentTimeMillis() / 1000.0,
        m_rotationSupplier.get(),
        m_swerveModulePositionSupplier.get());
    for (VisionIO visionIO : io) {
      visionIO.getVisionEst().ifPresent(est ->
        estimator.addVisionMeasurement(est.getPose(), est.getTimestamp())
      );
    }
    // In sim, fall back to drivetrain sim pose if module positions aren't simulated
    if (RobotBase.isSimulation() && m_poseSupplier != null) {
      m_field.setRobotPose(m_poseSupplier.get());
    } else {
      m_field.setRobotPose(estimator.getEstimatedPosition());
    }
  }

  @Override
  public void simulationPeriodic() {
    if(isReplay) {
      return;
    }
    // TODO: Get turret angle from turret subsystem
    if (m_turretCamSims.size() > 0) {
      for (PhotonVisionSimIO cameraSim : m_turretCamSims) {
        // The turret the camera is mounted on is rotated 5 degrees
        Rotation3d turretRotation = new Rotation3d(0, 0, Math.toRadians(5));
        Transform3d robotToCamera = new Transform3d(
                cameraSim.robotToCameraTrl.rotateBy(turretRotation),
                cameraSim.robotToCameraRot.rotateBy(turretRotation));
        sim.adjustCamera(cameraSim.getCameraSim(), robotToCamera);
      }
    }
    sim.update(m_poseSupplier.get());
  }

  public void setUpSim() {
    tagLayout = VisionConstants.kTagLayout;
    sim.addAprilTags(tagLayout);
    // Add this camera to the vision system simulation with the given robot-to-camera transform.
    for (PhotonVisionSimIO cameraSim : m_simCameras) {
      sim.addCamera(cameraSim.getCameraSim(), cameraSim.robotToCamera);
    }
  }

  public Pose2d getEstimatedPose() {
    return estimator.getEstimatedPosition();
  }
}
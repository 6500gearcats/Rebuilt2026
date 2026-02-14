package frc.robot.subsystems.vision.limelight;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionEstimate;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.vision.limelight.LimelightHelpers.LimelightTarget_Fiducial;

/**
 * VisionIO implementation for Limelight cameras.
 */
public class LimelightIO implements VisionIO {

    private String name;
    private Supplier<Rotation2d> rotationSupplier;
    private Supplier<Double> angularVelocitySupplier;
    @SuppressWarnings("unused")
    private boolean useMegaTag2;
    private boolean forPoseEstimation;

    /**
     * Creates a Limelight IO instance.
     *
     * @param theName                 limelight network table name
     * @param forPoseEstimation       true to use for pose estimation
     * @param rotationSupplier        robot rotation supplier
     * @param angularVelocitySupplier angular velocity supplier
     * @param useMegaTag2             true to use MegaTag2 pipeline
     */
    public LimelightIO(String theName, boolean forPoseEstimation, Supplier<Rotation2d> rotationSupplier,
            Supplier<Double> angularVelocitySupplier, boolean useMegaTag2) {
        name = theName;
        this.rotationSupplier = rotationSupplier;
        this.angularVelocitySupplier = angularVelocitySupplier;
        this.useMegaTag2 = useMegaTag2;
        this.forPoseEstimation = forPoseEstimation;
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public boolean forPoseEstimation() {
        return forPoseEstimation;
    }

    @Override
    public boolean hasTargets() {
        if (LimelightHelpers.getTV(name)) {
            return true;
        }
        return false;
    }

    @Override
    public double getBestYaw() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTX(name);
        }
        return 0;
    }

    @Override
    public double getBestPitch() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTY(name);
        }
        return 0;
    }

    @Override
    public double getBestRange() {
        if (LimelightHelpers.getTV(name)) {
            double x = (6.5 / (2 * getTargetAreaPercent())) / Math.tan(90 * Math.PI / 180);
            double distance = x / Math.cos(getBestYaw() * Math.PI / 180);
            return distance;
        }
        return 0;
    }

    public double getTargetAreaPercent() {
        if (LimelightHelpers.getTV(name)) {
            return LimelightHelpers.getTA(name);
        }
        return 0;
    }

    @Override
    public boolean hasChossenTarget(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (hasTargets()) {
            for (LimelightTarget_Fiducial result : results.targets_Fiducials) {
                if (result.fiducialID == fiducialID) {
                    return true;
                }
            }
        }
        return false;
    }

    @Override
    public double getChosenTargetYaw(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (hasChossenTarget(fiducialID)) {
            for (LimelightTarget_Fiducial result : results.targets_Fiducials) {
                if (result.fiducialID == fiducialID) {
                    return result.tx;
                }
            }
        }
        return 0;
    }

    @Override
    public double getChosenTargetPitch(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (hasChossenTarget(fiducialID)) {
            for (LimelightTarget_Fiducial result : results.targets_Fiducials) {
                if (result.fiducialID == fiducialID) {
                    return result.ty;
                }
            }
        }
        return 0;
    }

    @Override
    public double getChosenTargetSkew(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (hasChossenTarget(fiducialID)) {
            for (LimelightTarget_Fiducial result : results.targets_Fiducials) {
                if (result.fiducialID == fiducialID) {
                    return result.ts;
                }
            }
        }
        return 0;
    }

    @Override
    public double getChosenTargetRange(int fiducialID) {
        LimelightResults results = LimelightHelpers.getLatestResults(name);
        if (hasChossenTarget(fiducialID)) {
            for (LimelightTarget_Fiducial result : results.targets_Fiducials) {
                if (result.fiducialID == fiducialID) {
                    double x = (6.5 / (2 * result.ta)) / Math.tan(90 * Math.PI / 180);
                    double distance = x / Math.cos(getChosenTargetYaw(fiducialID) * Math.PI / 180);
                    return distance;
                }
            }
        }
        return 0;
    }

    @Override
    public Optional<VisionEstimate> getVisionEst() {
        // boolean useMegaTag2_ = true; // set to false to use MegaTag1
        boolean doRejectUpdate = false;

        if (useMegaTag2 == false) {
            /*
             * In 2024, most of the WPILib Ecosystem transitioned to a single-origin
             * coordinate system.
             * For 2024 and beyond, the origin of your coordinate system should always be
             * the "blue" origin.
             * FRC teams should always use botpose_wpiblue for pose-related functionality
             */
            LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(getName());
            // Optional<Alliance> alliance = DriverStation.getAlliance();
            // if (alliance.isPresent()) {
            // if (alliance.get() == Alliance.Red) {
            // mt1 = LimelightHelpers.getBotPoseEstimate_wpiRed("limelight-gcc");
            // }
            // else {
            // mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-gcc");
            // }
            if (mt1 != null) {
                if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                    if (mt1.rawFiducials[0].ambiguity > .7) {
                        doRejectUpdate = true;
                    }
                    if (mt1.rawFiducials[0].distToCamera > 3) {
                        doRejectUpdate = true;
                    }
                }
                if (mt1.tagCount == 0) {
                    doRejectUpdate = true;
                }

                if (!doRejectUpdate) {
                    return Optional.of(new VisionEstimate(mt1));
                }
            }
        } else if (useMegaTag2 == true) {
            /*
             * In 2024, most of the WPILib Ecosystem transitioned to a single-origin
             * coordinate system.
             * For 2024 and beyond, the origin of your coordinate system should always be
             * the "blue" origin.
             * FRC teams should always use botpose_wpiblue for pose-related functionality
             */

            // TODO: Get alliance as an instance variable instead of peroidic
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                if (alliance.get().equals(Alliance.Blue)) {
                    LimelightHelpers.SetRobotOrientation(getName(), rotationSupplier.get().getDegrees(), 0, 0, 0, 0, 0);
                } else {
                    LimelightHelpers.SetRobotOrientation(getName(), rotationSupplier.get().getDegrees() + 180, 0, 0, 0,
                            0, 0);
                }
            }
            // LimelightHelpers.setCameraPose_RobotSpace(getName(), -0.318, 0.177, 0.29, 0,
            // 0, 180);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(getName());
            if (mt2 != null) {
                if (Math.abs(angularVelocitySupplier.get()) > 720) // if our angular velocity is greater than 720
                                                                   // degrees per
                                                                   // second, ignore vision updates
                {
                    doRejectUpdate = true;
                }
                if (mt2.tagCount == 0) {
                    doRejectUpdate = true;
                }
                if (!doRejectUpdate) {
                    return Optional.of(new VisionEstimate(mt2));
                }
            }
        }
        return Optional.empty();
    }

}

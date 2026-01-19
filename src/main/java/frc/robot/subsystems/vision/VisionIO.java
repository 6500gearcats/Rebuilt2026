package frc.robot.subsystems.vision;

import java.util.Optional;

public interface VisionIO {
    public String getName();

    public boolean forPoseEstimation();

    public double getBestYaw();

    public double getBestPitch();

    public double getBestRange();

    public boolean hasTargets();

    public boolean hasChossenTarget(int fiducialID);

    public double getChosenTargetYaw(int fiducialID);

    public double getChosenTargetPitch(int fiducialID);

    public double getChosenTargetSkew(int fiducialID);

    public double getChosenTargetRange(int fiducialID);

    public Optional<VisionEstimate> getVisionEst();
}

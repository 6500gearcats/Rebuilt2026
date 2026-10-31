package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * Represents a 3D Pose Estimate.
 */
public class PoseEstimate {
    public Pose2d pose;
    public double timestampSeconds;
    public double latency;
    public int tagCount;
    public double tagSpan;
    public double avgTagDist;
    public double avgTagArea;

    public RawFiducial[] rawFiducials;
    public boolean isMegaTag2;

    /**
     * Instantiates a PoseEstimate object with default values
     */
    public PoseEstimate() {
        this.pose = new Pose2d();
        this.timestampSeconds = 0;
        this.latency = 0;
        this.tagCount = 0;
        this.tagSpan = 0;
        this.avgTagDist = 0;
        this.avgTagArea = 0;
        this.rawFiducials = new RawFiducial[]{};
        this.isMegaTag2 = false;
    }

    public PoseEstimate(Pose2d pose, double timestampSeconds, double latency,
                        int tagCount, double tagSpan, double avgTagDist,
                        double avgTagArea, RawFiducial[] rawFiducials, boolean isMegaTag2) {

        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.latency = latency;
        this.tagCount = tagCount;
        this.tagSpan = tagSpan;
        this.avgTagDist = avgTagDist;
        this.avgTagArea = avgTagArea;
        this.rawFiducials = rawFiducials;
        this.isMegaTag2 = isMegaTag2;
    }

}

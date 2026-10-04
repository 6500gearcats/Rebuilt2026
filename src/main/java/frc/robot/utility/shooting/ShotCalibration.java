package frc.robot.utility.shooting;

import java.util.Locale;
import java.util.Optional;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.TurretConstants;
import frc.robot.RobotStateMachine;

/** Dashboard tuning inputs and trial capture. Only the state machine commands shot motors. */
public final class ShotCalibration {
    private static final String ROOT = "Calibration/";
    private final RobotStateMachine stateMachine;
    private boolean enabled;
    private boolean motionEnabled;
    private double flywheelTrimRps;
    private CalibrationInputs inputs = new CalibrationInputs(0, 0, TurretConstants.kHoodMinPositionRotations, 0);
    private TrialSnapshot lastFeed;
    private int feedSequence;

    public ShotCalibration(RobotStateMachine stateMachine) {
        this.stateMachine = stateMachine;
        SmartDashboard.putBoolean(ROOT + "Enabled", false);
        SmartDashboard.putBoolean("Shot/Motion Compensation Enabled", false);
        SmartDashboard.putNumber("Shot/Flywheel Trim RPS", 0);
        SmartDashboard.putNumber(ROOT + "Distance Meters", 0);
        SmartDashboard.putNumber(ROOT + "Flywheel RPS", 0);
        SmartDashboard.putNumber(ROOT + "Hood Rotations", TurretConstants.kHoodMinPositionRotations);
        SmartDashboard.putNumber(ROOT + "TOF Seconds", 0);
        SmartDashboard.putString(ROOT + "Trial ID", "trial-001");
        SmartDashboard.putNumber(ROOT + "Attempts", 0);
        SmartDashboard.putNumber(ROOT + "Scored", 0);
        SmartDashboard.putString(ROOT + "Notes", "");
        SmartDashboard.putBoolean(ROOT + "Capture Trial", false);
        SmartDashboard.putString(ROOT + "Candidate Java Row", "Measure TOF before copying a row");
    }

    /** Read once before solution computation. Manual tuning is unavailable in autonomous. */
    public void readInputs(boolean canTune) {
        boolean nextEnabled = canTune && SmartDashboard.getBoolean(ROOT + "Enabled", false);
        if (nextEnabled != enabled) {
            stateMachine.getFlywheel().stopMotor();
            stateMachine.getHood().stop();
        }
        enabled = nextEnabled;
        if (!canTune) { SmartDashboard.putBoolean(ROOT + "Enabled", false); }
        motionEnabled = SmartDashboard.getBoolean("Shot/Motion Compensation Enabled", false);
        flywheelTrimRps = SmartDashboard.getNumber("Shot/Flywheel Trim RPS", 0);
        inputs = new CalibrationInputs(
                SmartDashboard.getNumber(ROOT + "Distance Meters", 0),
                SmartDashboard.getNumber(ROOT + "Flywheel RPS", 0),
                SmartDashboard.getNumber(ROOT + "Hood Rotations", TurretConstants.kHoodMinPositionRotations),
                SmartDashboard.getNumber(ROOT + "TOF Seconds", 0));
    }

    public boolean isEnabled() { return enabled; }
    public boolean isMotionEnabled() { return motionEnabled; }
    public double getFlywheelTrimRps() { return flywheelTrimRps; }
    public Optional<ShotSettings> manualSettings() {
        return inputs.manualSettings(TurretConstants.kHoodMinPositionRotations, TurretConstants.kHoodMaxPositionRotations);
    }

    public void adjustFlywheel(double deltaRps) {
        String key = enabled ? ROOT + "Flywheel RPS" : "Shot/Flywheel Trim RPS";
        double next = SmartDashboard.getNumber(key, 0) + deltaRps;
        SmartDashboard.putNumber(key, enabled ? Math.max(0, next) : next);
    }

    /** Hood buttons edit the calibration target only, in fine 0.005-rotation steps. */
    public void adjustHood(double deltaRotations) {
        if (!enabled) { return; }
        double next = SmartDashboard.getNumber(ROOT + "Hood Rotations", TurretConstants.kHoodMinPositionRotations)
                + deltaRotations;
        SmartDashboard.putNumber(ROOT + "Hood Rotations", Math.max(TurretConstants.kHoodMinPositionRotations,
                Math.min(TurretConstants.kHoodMaxPositionRotations, next)));
    }

    /** Disable clears tuning mode and operator trim; it retains trial annotations. */
    public void reset() {
        enabled = false;
        flywheelTrimRps = 0;
        SmartDashboard.putBoolean(ROOT + "Enabled", false);
        SmartDashboard.putNumber("Shot/Flywheel Trim RPS", 0);
        stateMachine.getFlywheel().stopMotor();
        stateMachine.getHood().stop();
    }

    /** Called only on the stopped-to-feeding transition, not for each scheduler tick. */
    public void recordFeedStart() {
        lastFeed = snapshot();
        feedSequence++;
        DataLogManager.log("ShotFeedStart sequence=" + feedSequence + " " + lastFeed);
    }

    /** Publish after subsystem control so requested/actual values belong to this cycle. */
    public void publishAndCapture() {
        SmartDashboard.putNumber(ROOT + "Pose Distance Meters", stateMachine.getTurretPose().getTranslation()
                .getDistance(stateMachine.getHubPose().getTranslation()));
        SmartDashboard.putNumber(ROOT + "Requested RPS", stateMachine.getFlywheel().getReqSpeed());
        SmartDashboard.putNumber(ROOT + "Actual RPS", stateMachine.getFlywheel().getSpeed());
        SmartDashboard.putNumber(ROOT + "Requested Hood Rotations", stateMachine.getHood().getRequestedPositionRotations());
        SmartDashboard.putNumber(ROOT + "Actual Hood Rotations", stateMachine.getHood().getAbsolutePositionRotations());
        SmartDashboard.putBoolean(ROOT + "Flywheel Ready", stateMachine.getFlywheel().isUpToSpeed());
        SmartDashboard.putBoolean(ROOT + "Hood Ready", stateMachine.getHood().isAtPosition());
        SmartDashboard.putNumber(ROOT + "Battery Volts", RobotController.getBatteryVoltage());
        SmartDashboard.putBoolean("Shot/Can Feed", stateMachine.canFeedShot());
        if (SmartDashboard.getBoolean(ROOT + "Capture Trial", false)) {
            SmartDashboard.putBoolean(ROOT + "Capture Trial", false);
            captureTrial();
        }
    }

    /**
     * Preserve feed-start settings while adding post-trial video timing/results.
     * A snapshot without feeding is useful for diagnosis but generates no candidate row.
     */
    private void captureTrial() {
        String trialId = SmartDashboard.getString(ROOT + "Trial ID", "");
        boolean hasFeed = lastFeed != null && lastFeed.trialId().equals(trialId);
        TrialSnapshot trial = hasFeed ? lastFeed : snapshot();
        double tof = SmartDashboard.getNumber(ROOT + "TOF Seconds", 0);
        double attempts = SmartDashboard.getNumber(ROOT + "Attempts", 0);
        double scored = SmartDashboard.getNumber(ROOT + "Scored", 0);
        String notes = SmartDashboard.getString(ROOT + "Notes", "").replace('\n', ' ').replace('\r', ' ');
        CalibrationInputs measured = new CalibrationInputs(trial.measuredDistanceMeters(),
                trial.requestedRps(), trial.requestedHoodRotations(), tof);
        Optional<String> row = hasFeed && trial.manual() && trial.ready()
                && Double.isFinite(attempts) && Double.isFinite(scored)
                && attempts >= 1 && attempts == Math.rint(attempts)
                && scored >= 0 && scored <= attempts && scored == Math.rint(scored)
                ? measured.javaRow(TurretConstants.kHoodMinPositionRotations, TurretConstants.kHoodMaxPositionRotations)
                : Optional.empty();
        String candidate = row.orElse("No candidate: need a ready manual feed trial, positive TOF, and valid outcome counts");
        SmartDashboard.putString(ROOT + "Candidate Java Row", candidate);
        DataLogManager.log(String.format(Locale.ROOT,
                "ShotTrial capturedAt=%.6f feedSnapshot=%s %s tofSeconds=%.6f scored=%.0f attempts=%.0f notes=%s candidate=%s",
                Timer.getFPGATimestamp(), hasFeed, trial, tof, scored, attempts, notes, candidate));
    }

    private TrialSnapshot snapshot() {
        return new TrialSnapshot(SmartDashboard.getString(ROOT + "Trial ID", "").replace('\n', ' ').replace('\r', ' '),
                Timer.getFPGATimestamp(), enabled, inputs.distanceMeters(),
                stateMachine.getTurretPose().getTranslation().getDistance(stateMachine.getHubPose().getTranslation()),
                stateMachine.getFlywheel().getReqSpeed(), stateMachine.getFlywheel().getSpeed(),
                stateMachine.getHood().getRequestedPositionRotations(), stateMachine.getHood().getAbsolutePositionRotations(),
                stateMachine.canFeedShot(), enabled ? 0 : flywheelTrimRps, RobotController.getBatteryVoltage());
    }

    private record TrialSnapshot(String trialId, double feedCommandTimeSeconds, boolean manual,
            double measuredDistanceMeters, double poseDistanceMeters, double requestedRps, double actualRps,
            double requestedHoodRotations, double actualHoodRotations, boolean ready,
            double flywheelTrimRps, double batteryVolts) {}
}

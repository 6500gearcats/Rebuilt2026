# Code Issues & Suggested Fixes

This document catalogs known problems in the codebase, ordered by severity.
The primary symptom — **loop overrun warnings** — means the 20ms robot loop is taking longer than 20ms to complete. WPILib prints a warning to the Driver Station console every time this happens. Sustained overruns cause jerky driving, missed vision frames, and unreliable shooting.

---

## Table of Contents

1. [What Is a Loop Overrun?](#what-is-a-loop-overrun)
2. [Critical — Fix These First](#critical--fix-these-first)
3. [High Severity](#high-severity)
4. [Medium Severity](#medium-severity)
5. [Low Severity](#low-severity)
6. [Summary Table](#summary-table)

---

## What Is a Loop Overrun?

The WPILib scheduler runs every 20ms (50 times per second). Each cycle it:
1. Calls `periodic()` on every registered subsystem
2. Calls `execute()` on every running command
3. Evaluates all triggers and button bindings

If any of those operations take more than 20ms total, the next cycle starts late. You will see:

```
Loop time of 0.025s is greater than the expected 0.02s
```

on the Driver Station console. The most common causes are:
- **Blocking operations** (network I/O, disk reads)
- **Heavy math** that should be cached or done less often
- **GC pressure** from allocating objects every loop cycle
- **Excessive SmartDashboard/NetworkTables writes** (each write serializes data and sends a UDP packet)

---

## Critical — Fix These First

These are the most likely causes of the loop overrun messages you are seeing.

---

### C-1: `System.gc()` Called Every 100ms in `robotPeriodic()`

**File:** `Robot.java` — `robotPeriodic()` method
**Severity:** Critical

**The problem:**
```java
// Called every 20ms. Timer fires every 100ms = GC every 5 loops.
if (m_gcTimer.advanceIfElapsed(0.1)) {
    System.gc();
}
```

Java's garbage collector stops the world when it runs. Even a minor GC can take 5–15ms on the roboRIO. A major GC can exceed 50ms. Calling `System.gc()` explicitly every 100ms *guarantees* periodic loop overruns.

**Why it was added:** The intent is to keep the heap clean. However, the JVM GC is smarter than manual calls — it runs when it needs to, not on a fixed schedule. Forcing it more often causes more pauses, not fewer.

**Fix:** Delete both `System.gc()` calls and the `m_gcTimer` field entirely.

```java
// DELETE these lines from robotPeriodic():
if (m_gcTimer.advanceIfElapsed(0.1)) {
    System.gc();
}
```

If you want to reduce GC pressure, the real fix is to stop allocating objects in hot paths (see C-3 and H-5 below).

---

### C-2: Excessive SmartDashboard Writes in `RobotStateMachine.periodic()`

**File:** `RobotStateMachine.java` — `periodic()` method
**Severity:** Critical

**The problem:**
`RobotStateMachine.periodic()` runs 10+ SmartDashboard/NetworkTables writes *plus* calls `updateTargetPose()` and `refreshPoseFromVision()` every single loop cycle. Each NT write serializes an object and dispatches a UDP packet. Doing 10+ in a row adds up fast.

Specifically per loop:
- `SmartDashboard.putString(...)` × 3
- `SmartDashboard.putBoolean(...)` × 1
- `SmartDashboard.putNumber(...)` × 4+
- `posePublisher.set(pose)` — NT struct publish
- `turretPosePublisher.set(turretPose)` — NT struct publish
- `targetPosePublisher.set(targetPose)` — NT struct publish (inside `updateTargetPose()`)

**Fix:** Rate-limit telemetry to once every 100ms. Only the values that *control* the robot (not display values) should run every loop.

```java
// Add a timer field:
private final Timer m_telemetryTimer = new Timer();

// In periodic(), wrap all SmartDashboard puts:
if (m_telemetryTimer.advanceIfElapsed(0.1)) {  // 10 Hz instead of 50 Hz
    SmartDashboard.putString("Alliance", alliance.toString());
    SmartDashboard.putString("Robot State", getState().toString());
    SmartDashboard.putNumber("Target Distance", distanceToHub);
    // etc.
}

// Pose publishers for field visualization can also be 10 Hz:
if (m_telemetryTimer.advanceIfElapsed(0.1)) {
    posePublisher.set(pose);
    turretPosePublisher.set(turretPose);
}
```

Control-critical values (like `targetPose` used by `AlignTurretToHub`) should still update every loop, but the display-only values should not.

---

### C-3: Object Allocation in `periodic()` — GC Pressure

**File:** `RobotStateMachine.java` — `periodic()` and `updateTargetPose()`
**Severity:** Critical

**The problem:**
Every loop cycle, new Java objects are created and immediately discarded:

```java
// In periodic() — runs 50x/second:
turretPose = new Pose2d(...)          // new object
    .rotateAround(..., new Rotation2d(0));  // another new object

// In updateTargetPose() — also runs 50x/second:
ChassisSpeeds speeds = getChassisSpeeds();   // new object
Pose2d predictedTarget = new Pose2d(...);   // new object
```

50 objects per second × 3 fields = 150+ short-lived objects/second. The JVM has to GC all of these, which causes the pauses you see.

**Fix:** Pre-allocate reusable objects and update them in place, or cache computed values that don't change every loop.

```java
// Declare once as fields:
private Pose2d m_turretPose = new Pose2d();
private Pose2d m_predictedTarget = new Pose2d();

// In periodic(), reuse them (WPILib Pose2d is immutable, so you need to
// re-assign rather than mutate — but reducing allocation frequency still helps):
// Only recompute turretPose when the robot pose has meaningfully changed:
if (poseHasChanged()) {
    m_turretPose = new Pose2d(...);  // still allocates but less often
}
```

The most impactful change is to move the ballistic physics calculation (`updateTargetPose()`) to run at 10 Hz instead of 50 Hz — the target moves slowly enough that 10 Hz is sufficient for the turret PID to track it.

---

### C-4: `AlignTurretToHub.execute()` Has 6 SmartDashboard Writes Per Loop

**File:** `commands/AlignTurretToHub.java` — `execute()` method
**Severity:** Critical

**The problem:**
This command runs continuously while the trigger is held. Its `execute()` method fires every 20ms and contains 6 SmartDashboard writes plus complex pose math:

```java
SmartDashboard.putNumber("Target Angle Unclamped", ...);
SmartDashboard.putNumber("Target Angle Clamped", ...);
SmartDashboard.putNumber("Robot Angle", ...);
SmartDashboard.putNumber("Turret Setpoint", ...);
SmartDashboard.putNumber("distance to hub", ...);
SmartDashboard.putNumber("Required Speed", ...);
```

When shooting, this command is always running, meaning these 6 puts fire every loop cycle on top of everything else.

**Fix:** Wrap all debug puts in a rate-limited timer or a `SmartDashboard.putBoolean("Debug Mode", ...)` flag:

```java
private final Timer m_debugTimer = new Timer();

@Override
public void execute() {
    // ... (all pose math stays every loop — it drives the turret) ...
    turret.setPosition(clampedAngle);  // must run every loop

    // Debug output only 10x/second:
    if (m_debugTimer.advanceIfElapsed(0.1)) {
        SmartDashboard.putNumber("Target Angle", clampedAngle);
        SmartDashboard.putNumber("distance to hub", distance);
        // etc.
    }
}
```

---

## High Severity

---

### H-1: 4 Sequential CAN Reads in `getChassisSpeeds()` Called Every Loop

**File:** `RobotStateMachine.java` — `getChassisSpeeds()` method
**Severity:** High

**The problem:**
```java
public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getKinematics().toChassisSpeeds(
        drivetrain.getModule(0).getCurrentState(),   // CAN read
        drivetrain.getModule(1).getCurrentState(),   // CAN read
        drivetrain.getModule(2).getCurrentState(),   // CAN read
        drivetrain.getModule(3).getCurrentState());  // CAN read
}
```

This is called from `updateTargetPose()` every loop. Each `getCurrentState()` call on a swerve module reads the latest signal from the CAN bus. Four sequential CAN reads adds latency.

**Fix:** Cache the chassis speeds. The drivetrain's `periodic()` should compute and store `ChassisSpeeds` once per loop, then `getChassisSpeeds()` just returns the cached value.

```java
// In CommandSwerveDrivetrain.periodic():
m_cachedSpeeds = m_kinematics.toChassisSpeeds(getState().ModuleStates);

// getChassisSpeeds() becomes:
public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getCachedSpeeds();  // no CAN reads
}
```

---

### H-2: Vision Loop Calls `getVisionEst()` Twice Per Camera

**File:** `subsystems/vision/Vision.java` — `periodic()` method
**Severity:** High

**The problem:**
Inside the camera loop, `getVisionEst()` is called twice for some cameras — once for the pose update, and again inside the conditional for publishing:

```java
for (VisionIO visionIO : m_visionOdometryCams) {
    visionIO.getVisionEst().ifPresent(est -> {
        estimator.addVisionMeasurement(...);  // call 1
    });
    if (visionIO.getName().contains("gcc")) {
        visionIO.getVisionEst().ifPresent(est -> gccPub.set(...));  // call 2
    }
}
```

Each `getVisionEst()` reads from NetworkTables (for Limelight) or processes a PhotonVision result. Double-calling it is wasted work.

**Fix:** Store the result once:

```java
for (VisionIO visionIO : m_visionOdometryCams) {
    var result = visionIO.getVisionEst();
    result.ifPresent(est -> estimator.addVisionMeasurement(...));
    if (visionIO.getName().contains("gcc")) {
        result.ifPresent(est -> gccPub.set(est.getPose()));
    }
}
```

---

### H-3: Vision Accepts All Measurements Without Quality Filtering

**File:** `subsystems/vision/Vision.java` — `periodic()` method
**Severity:** High

**The problem:**
Every vision measurement is blindly added to the pose estimator with no validation:

```java
visionIO.getVisionEst().ifPresent(est -> {
    estimator.addVisionMeasurement(est.getPose(), est.getTimestamp());
    // NOTE: quality filtering was commented out
});
```

A low-confidence single-tag detection with high ambiguity can corrupt the pose estimate significantly, causing the turret to aim at the wrong location.

**Fix:** Apply distance-based and ambiguity-based filtering:

```java
visionIO.getVisionEst().ifPresent(est -> {
    // Reject if the reported pose is far from the current estimate (likely bad tag)
    double distance = est.getPose().getTranslation()
        .getDistance(estimator.getEstimatedPosition().getTranslation());
    if (distance > 1.0) return;  // reject if >1m jump

    // Scale trust based on distance from tag
    double tagDist = est.targetsUsed().get(0).getBestCameraToTarget()
        .getTranslation().getNorm();
    Vector<N3> dynamicStdDev = VecBuilder.fill(
        0.1 * tagDist, 0.1 * tagDist, Units.degreesToRadians(5 * tagDist));
    estimator.addVisionMeasurement(est.getPose(), est.getTimestamp(), dynamicStdDev);
});
```

---

### H-4: Turret SmartDashboard Writes Every Loop in `periodic()`

**File:** `subsystems/turret/Turret.java` — `periodic()` method
**Severity:** High

**The problem:**
```java
@Override
public void periodic() {
    SmartDashboard.putBoolean("switch on or off", m_switch.get());   // hardware read
    SmartDashboard.putNumber("Motor Position", getMotorPosition());   // CAN read
    SmartDashboard.putNumber("Turret Position", getConvertedTurretPosition());
    SmartDashboard.putNumber("Robot Rot in Deg", ...);
}
```

Four SmartDashboard writes and a hardware/CAN read every 20ms. Same pattern in `Flywheel.periodic()` with 8 writes.

**Fix:** Rate-limit to 10 Hz (same approach as C-2 above). The turret's actual motor commands go through `setPosition()` which is called by commands — those should remain every loop. Only telemetry should be rate-limited.

---

### H-5: `CoolSnurbo` Does Not Declare Flywheel as a Requirement

**File:** `commands/CoolSnurbo.java`
**Severity:** High

**The problem:**
```java
public CoolSnurbo(Flywheel m_flywheel) {
    this.m_flywheel = m_flywheel;
    // Use addRequirements() here to declare subsystem dependencies.
    // ^ This comment exists but addRequirements() was never called
}
```

Without `addRequirements(flywheel)`, the WPILib scheduler does not know that `CoolSnurbo` uses the `Flywheel`. This means:
- Two `CoolSnurbo` commands could run simultaneously on the same flywheel
- `CoolSnurbo` running inside `ShootingSequenceUTS` could be preempted unexpectedly by another command that *does* declare the flywheel requirement

**Fix:**
```java
public CoolSnurbo(Flywheel m_flywheel) {
    this.m_flywheel = m_flywheel;
    addRequirements(m_flywheel);
}
```

> **Note:** Adding this requirement will cause `ShootingSequenceUTS`'s parallel group to have two commands both requiring `Flywheel` (`UpToSpeedHopperShoot` and `CoolSnurbo`). WPILib will reject this. You need to refactor — either merge the snurbo logic into `UpToSpeedHopperShoot`, or make `CoolSnurbo` an `InstantCommand` that just sets a flag rather than requiring the subsystem continuously.

---

### H-6: Climber Motor Is Permanently Disabled

**File:** `subsystems/Climber.java` — `setMotorSpeed()` method
**Severity:** High

**The problem:**
```java
public void setMotorSpeed(double speed) {
    // if ((getMotorPosition() < 1.2)) {
    // if (speed < 0) { speed = 0; } }
    // m_motor.set(speed);
}
```

The motor call is commented out. `ClimbPole` commands are bound to buttons and appear functional, but the climber never moves. The driver pressing the climb button will see nothing happen.

**Fix:** Either uncomment and restore the motor call (with appropriate safety checks), or remove the `ClimbPole` button bindings and add a visible comment that climbing is not yet implemented.

---

### H-7: `RobotStateMachine` Singleton Is Not Thread-Safe

**File:** `RobotStateMachine.java` — `getInstance()` method
**Severity:** High

**The problem:**
```java
public static RobotStateMachine getInstance() {
    if (instance == null) {
        instance = new RobotStateMachine();  // race condition here
    }
    return instance;
}
```

PhotonVision and Limelight run their network processing on background threads. If a background thread calls `getInstance()` at the same moment as the main robot thread during startup, two instances could be created.

**Fix:** Use eager initialization (create the instance at class load time):

```java
private static final RobotStateMachine instance = new RobotStateMachine();

public static RobotStateMachine getInstance() {
    return instance;
}
```

---

## Medium Severity

---

### M-1: `Turret.java` PID Slot 1 Gains Default to Zero

**File:** `subsystems/turret/Turret.java` — constructor
**Severity:** Medium

**The problem:**
```java
slot1Configs.kV = SmartDashboard.getNumber("kV", 0);
slot1Configs.kA = SmartDashboard.getNumber("kA", 0);
slot1Configs.kP = SmartDashboard.getNumber("kP", 0);
```

If the SmartDashboard values have never been set (fresh Driver Station session, new laptop, etc.), all PID gains for Slot 1 will be zero, making the turret uncontrollable in that mode.

**Fix:** Either hard-code the known-good values as defaults (same as Slot 2's values), or move all tuning values to `Constants.java`:

```java
slot1Configs.kV = SmartDashboard.getNumber("kV", TurretConstants.kV);  // fallback to constant
```

---

### M-2: Turret `setPosition()` Does Not Bounds-Check Angles

**File:** `subsystems/turret/Turret.java` — `setPosition()` method
**Severity:** Medium

**The problem:**
`setSpeed()` checks bounds and clamps speed to zero at the limits. But `setPosition()` (used by `AlignTurretToHub`) sends arbitrary position targets directly to the motor without checking if the target is within the 0–55 rotation range:

```java
public void setPosition(double degrees) {
    double rotations = (degrees / 360.0) * turretRotations;
    m_motor.setControl(m_positionOut.withPosition(rotations));
    // No bounds check here
}
```

**Fix:** Clamp the target before commanding:

```java
public void setPosition(double degrees) {
    double rotations = (degrees / 360.0) * turretRotations;
    rotations = MathUtil.clamp(rotations, 2, 53);  // stay within soft limits
    m_motor.setControl(m_positionOut.withPosition(rotations));
}
```

---

### M-3: `RobotConfig` Load Failure Is Silent

**File:** `Constants.java` — static initializer
**Severity:** Medium

**The problem:**
```java
static {
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        e.printStackTrace();  // printed to console, but config is null
    }
}
```

If the GUI settings file is missing (common on a freshly imaged RoboRIO), `config` remains `null`. Any PathPlanner code that uses `config` will throw a `NullPointerException` mid-match with no meaningful error message.

**Fix:** Provide a hard-coded fallback config:

```java
static {
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        System.err.println("[Constants] Failed to load RobotConfig from GUI — using defaults!");
        config = new RobotConfig(/* default values */);
    }
}
```

---

### M-4: Static Vision Standard Deviations Trust Single-Tag and Multi-Tag Equally

**File:** `subsystems/vision/Vision.java` — constants
**Severity:** Medium

**The problem:**
```java
private static final Vector<N3> m_visionStndDev = VecBuilder.fill(0.1, 0.1, 0.1);
```

A detection with one tag has much higher pose ambiguity than a detection with three tags. Applying the same low standard deviation to both means single-tag detections are over-trusted and can corrupt the pose estimate.

**Fix:** Use dynamic standard deviations that scale with distance and number of targets (see H-3 above for the full code example).

---

### M-5: `UpToSpeedHopperShoot` Reads SmartDashboard Every Execute

**File:** `commands/UpToSpeedHopperShoot.java` — `execute()` method
**Severity:** Medium

**The problem:**
```java
m_Flywheel.setSpeed(SmartDashboard.getNumber("Shoot Speed", 0));
```

This reads from NetworkTables every 20ms while the command is running. Additionally, if "Shoot Speed" is never set on the dashboard, it defaults to 0 and the robot fires with the flywheel stopped.

**Fix:** Read once in `initialize()` and cache the value, or use the `RangeFinder` directly:

```java
private double m_targetSpeed;

@Override
public void initialize() {
    // Read once — speed doesn't need to change mid-shot
    m_targetSpeed = SmartDashboard.getNumber("Shoot Speed", RangeFinder.getSpeed(currentDistance));
}

@Override
public void execute() {
    m_Flywheel.setSpeed(m_targetSpeed);  // no NT read in loop
    // ...
}
```

---

## Low Severity

---

### L-1: Dead Code in `Turret.java`

**File:** `subsystems/turret/Turret.java` — constructor
**Line:** `m_motor.getConfigurator();` — the return value is discarded and the call does nothing.

Remove it.

---

### L-2: Double Semicolon in `RunIntake.java`

**File:** `commands/RunIntake.java`

```java
private double speed;;   // double semicolon
```

Remove the extra semicolon. It compiles but is a sign of an edit error.

---

### L-3: Duplicate Imports in `Vision.java`

**File:** `subsystems/vision/Vision.java`

`NetworkTableInstance` and `StructPublisher` are imported twice. Remove the duplicates. Your IDE will flag these as warnings.

---

### L-4: Flywheel `TODO` — No Static Friction Compensation

**File:** `subsystems/turret/Flywheel.java`

```java
// TODO: Add a constant Spin to the motors to not have to fight static friction
```

The `kS` (static friction) feedforward gain in the CTRE configuration addresses this at the hardware level. Verify that the configured `kS = 0.3087` is sufficient. If the flywheel stalls at low speed commands, increase `kS` in `Constants.java` rather than adding a separate idle-spin command.

---

### L-5: Intake Deploy Has No Soft Limit

**File:** `subsystems/intake/Intake.java` — `deployIntake()` method
**Severity:** Low

`deployIntake()` runs the deploy motor at a given speed with no position limit. If something jams or the command is left running, the mechanism can over-extend. Add a TalonFX soft limit or a timer-based cutoff.

---

## Summary Table

| ID | File | Issue | Severity |
|----|------|-------|----------|
| C-1 | `Robot.java` | `System.gc()` every 100ms causes GC pauses | **Critical** |
| C-2 | `RobotStateMachine.java` | 10+ SmartDashboard writes every loop | **Critical** |
| C-3 | `RobotStateMachine.java` | New `Pose2d`/`Rotation2d` objects created every loop | **Critical** |
| C-4 | `AlignTurretToHub.java` | 6 SmartDashboard writes in `execute()` while shooting | **Critical** |
| H-1 | `RobotStateMachine.java` | 4 CAN reads in `getChassisSpeeds()` every loop | High |
| H-2 | `Vision.java` | `getVisionEst()` called twice per camera per loop | High |
| H-3 | `Vision.java` | No quality filtering on vision measurements | High |
| H-4 | `Turret.java` / `Flywheel.java` | SmartDashboard writes every loop in `periodic()` | High |
| H-5 | `CoolSnurbo.java` | Missing `addRequirements()` — scheduler blind to flywheel use | High |
| H-6 | `Climber.java` | Motor permanently commented out — climb never works | High |
| H-7 | `RobotStateMachine.java` | Non-thread-safe singleton | High |
| M-1 | `Turret.java` | PID Slot 1 gains default to zero if dashboard not set | Medium |
| M-2 | `Turret.java` | `setPosition()` doesn't bounds-check angle | Medium |
| M-3 | `Constants.java` | `RobotConfig` null if load fails | Medium |
| M-4 | `Vision.java` | Static standard deviations for all vision measurements | Medium |
| M-5 | `UpToSpeedHopperShoot.java` | SmartDashboard read every `execute()` loop | Medium |
| L-1 | `Turret.java` | Dead call `m_motor.getConfigurator()` | Low |
| L-2 | `RunIntake.java` | Double semicolon `private double speed;;` | Low |
| L-3 | `Vision.java` | Duplicate imports | Low |
| L-4 | `Flywheel.java` | `kS` TODO — static friction compensation | Low |
| L-5 | `Intake.java` | Deploy motor has no soft limit | Low |

---

## Recommended Fix Order

1. **C-1** — Delete `System.gc()`. One-line fix, immediate loop overrun relief.
2. **C-2** — Rate-limit `RobotStateMachine.periodic()` telemetry to 10 Hz.
3. **C-4** — Rate-limit `AlignTurretToHub.execute()` debug puts to 10 Hz.
4. **H-4** — Rate-limit `Turret.periodic()` and `Flywheel.periodic()` telemetry.
5. **H-1** — Cache `ChassisSpeeds` in the drivetrain and use the cached value.
6. **H-5** — Fix `CoolSnurbo.addRequirements()` and resolve the parallel command conflict.
7. **H-3 + M-4** — Add vision quality filtering and dynamic standard deviations.
8. **M-2** — Add bounds clamping to `Turret.setPosition()`.
9. **H-6** — Decide on climber: restore or remove bindings.

Steps 1–4 alone should eliminate the majority of loop overruns.

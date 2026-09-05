# Gearcats Rebuilt2026 — Master Development Plan

**Branch:** `IntegrateHackCode` / `leto`  
**Working directory:** `c:\projects\Gearcats\Rebuilt2026\Rebuilt2026`  
**Updated:** 2026-09-05  
**Covers:** Loop overrun fixes · Dead code · Limelight deprecation · Logging · Shoot-on-the-move · Hackbots integration · Lead compensation

---

## Plan Overview

Nine ordered stages. Stages 0–3 fix and improve the existing Rebuilt2026 code. Stages 4–6 are the Hackbots integration. Stages 7–8 are enhancements and hardware. **Stage 0 must be done before anything else** — the robot is currently producing loop overrun warnings that will cause jerky driving and unreliable shooting.

| Stage | Name | Risk | When | Dependency |
|-------|------|------|------|------------|
| **0** | **Critical Performance Fixes (Loop Overruns)** | **LOW** | **First** | **None** |
| 1 | Dead Code Removal & Limelight Deprecation | LOW | After S0 | Stage 0 |
| 2 | Logging & Telemetry Improvements | LOW | After S0 | Stage 0 |
| 3 | Shoot-on-the-Move Accuracy Fixes | MEDIUM | Before integration | Stage 1 |
| 4 | Hackbots Integration — Files & Compile | MEDIUM | After S1–S3 | Stage 1, 3 |
| 5 | Hackbots Integration — State Machine Wire-Up | HIGH | After S4 compiles | Stage 4 |
| 6 | Hackbots Integration — RobotContainer & Auto | HIGH | After S5 | Stage 5 |
| 7 | Lead Compensation Enhancement | MEDIUM | After S6 verified | Stage 6 |
| 8 | Hardware Wiring, Tuning & Measurement | HARDWARE | On assembled robot | Stages 4–7 |

> **Source of Stage 0 issues:** `ISSUES.md` — the robot is showing WPILib loop overrun warnings (`Loop time of 0.025s is greater than the expected 0.02s`). The four critical-severity issues below are the primary causes. Fixing them alone should eliminate the majority of overruns. All 20 issues from `ISSUES.md` are distributed across the appropriate stages below.

---

## Stage 0: Critical Performance Fixes (Loop Overruns)

**Risk: LOW — Rate-limiting and one-line deletions only — No control logic changes**

The WPILib scheduler runs every 20ms. If any single loop cycle takes more than 20ms total (across all `periodic()` calls and running commands), WPILib prints a warning to the Driver Station console and the next cycle starts late. Sustained overruns cause jerky driving, missed vision frames, and unreliable shooting. Do these first.

### ISSUES.md Cross-Reference

| ID | Issue | Fix Location |
|----|-------|-------------|
| C-1 | `System.gc()` every 100ms — guarantees GC pauses of 5–50ms | `Robot.java` |
| C-2 | 10+ SmartDashboard writes every loop in `RobotStateMachine.periodic()` | `RobotStateMachine.java` |
| C-3 | New `Pose2d`/`Rotation2d` objects created every loop — GC pressure | `RobotStateMachine.java` |
| H-1 | 4 CAN reads in `getChassisSpeeds()` called every loop | `RobotStateMachine.java` + drivetrain |
| H-7 | Non-thread-safe singleton — race condition at startup | `RobotStateMachine.java` |
| M-2 | `setPosition()` sends arbitrary angles without bounds check | `Turret.java` |
| M-3 | `RobotConfig` remains `null` if GUI settings file missing | `Constants.java` |

---

### 0-1: Delete `System.gc()` (Issue C-1)

**File:** `src/main/java/frc/robot/Robot.java`

The most impactful single-line fix. Java's garbage collector runs when it needs to — forcing it every 100ms causes more frequent stop-the-world pauses, not fewer.

```java
// DELETE these lines from robotPeriodic():
if (m_gcTimer.advanceIfElapsed(0.1)) {
    System.gc();
}

// Also delete the m_gcTimer field declaration from the class body
```

Expected result: eliminates 5–50ms GC pauses that were guaranteed to fire every 5 loops.

---

### 0-2: Rate-limit `RobotStateMachine.periodic()` telemetry (Issues C-2, C-3)

**File:** `src/main/java/frc/robot/RobotStateMachine.java`

Currently 10+ SmartDashboard writes plus three NT struct publishes fire every 20ms. SmartDashboard writes are not free — each serializes an object and dispatches a UDP packet.

```java
// Add as a class field:
private final Timer m_telemetryTimer = new Timer();

// In periodic(), wrap all display-only SmartDashboard puts:
if (m_telemetryTimer.advanceIfElapsed(0.1)) {   // 10 Hz
    SmartDashboard.putString("Robot/State", getState().toString());
    SmartDashboard.putString("Robot/FieldZone", getZone().toString());
    SmartDashboard.putNumber("Robot/DistToHubM", distanceToHub);
    SmartDashboard.putNumber("Robot/FieldVelX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot/FieldVelY", speeds.vyMetersPerSecond);
    SmartDashboard.putBoolean("Robot/IsFacingHub", isFacing);
    // ... all other display-only puts
    posePublisher.set(currentPose);         // field viz can be 10 Hz
    turretPosePublisher.set(turretPose);
}
// targetPosePublisher stays every loop — it drives AlignTurretToHub
```

Also reduce `updateTargetPose()` to run at 10 Hz — the predicted target moves slowly enough that the turret PID can track a 10 Hz update without loss of accuracy:

```java
if (m_telemetryTimer.advanceIfElapsed(0.1)) {
    updateTargetPose();
}
```

---

### 0-3: Cache `ChassisSpeeds` — eliminate 4 CAN reads per loop (Issue H-1)

**File:** `src/main/java/frc/robot/RobotStateMachine.java` + `CommandSwerveDrivetrain.java`

`getChassisSpeeds()` currently calls `getCurrentState()` on all four swerve modules every loop — 4 CAN bus reads. The drivetrain's own `periodic()` already has this data. Cache it there.

```java
// In CommandSwerveDrivetrain.java periodic():
m_cachedSpeeds = m_kinematics.toChassisSpeeds(getState().ModuleStates);

// getChassisSpeeds() in RobotStateMachine becomes:
public ChassisSpeeds getChassisSpeeds() {
    return drivetrain.getCachedSpeeds();   // no CAN reads
}
```

---

### 0-4: Fix non-thread-safe singleton (Issue H-7)

**File:** `src/main/java/frc/robot/RobotStateMachine.java`

```java
// BEFORE — race condition if background thread calls getInstance() during startup:
public static RobotStateMachine getInstance() {
    if (instance == null) {
        instance = new RobotStateMachine();
    }
    return instance;
}

// AFTER — eager initialization, guaranteed single instance:
private static final RobotStateMachine instance = new RobotStateMachine();

public static RobotStateMachine getInstance() {
    return instance;
}
```

---

### 0-5: Add bounds clamp to `Turret.setPosition()` (Issue M-2)

**File:** `src/main/java/frc/robot/subsystems/turret/Turret.java`

A bad angle from `AlignTurretToHub` could currently drive the turret into its hard stop.

```java
public void setPosition(double degrees) {
    double rotations = (degrees / 360.0) * turretRotations;
    rotations = MathUtil.clamp(rotations, 2, 53);   // stay within soft limits
    m_motor.setControl(m_positionOut.withPosition(rotations));
}
```

Import: `edu.wpi.first.math.MathUtil`

---

### 0-6: Add `RobotConfig` null safety (Issue M-3)

**File:** `src/main/java/frc/robot/Constants.java`

If the PathPlanner GUI settings file is missing (common on a freshly-imaged RoboRIO), `config` remains `null` and any auto command throws a `NullPointerException` mid-match.

```java
static {
    try {
        config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
        System.err.println("[Constants] RobotConfig load failed — using defaults!");
        config = new RobotConfig(/* robot-specific defaults here */);
    }
}
```

---

### Stage 0 Verification ✓

1. Deploy and drive for 30 seconds — confirm zero "Loop time greater than 0.02s" warnings in DS console
2. `getChassisSpeeds()` returns same values as before (verify by logging `Robot/FieldVelX`)
3. SmartDashboard updates visibly at 10 Hz (still perceptible, not sluggish)
4. Singleton: `RobotStateMachine.getInstance()` returns same object on repeated calls

---

## Stage 1: Dead Code Removal & Limelight Deprecation

**Risk: LOW — Purely subtractive — No functional changes**

Incorporates ISSUES.md items: **L-1, L-2, L-3, H-5, H-6**

### 1A — Fix Compile Errors

#### 1A-1: Declare `nextPose` as a local variable in `RobotStateMachine.java`

After the Bug 1 fix, `nextPose` is used inside the `updateTargetPose()` loop but never declared — compile error blocking the branch from building.

- Add `Pose2d nextPose = new Pose2d();` at the top of the `for` loop
- Remove dead variables `predX` and `predY` (lines 249–250 — orphaned after the fix)

### 1B — Remove Dead Code

#### 1B-1: Remove dead `RangeFinder` instance from `RobotContainer.java`

`private final RangeFinder rangeFinder = new RangeFinder();` is never used — the class is only called statically. Delete the field declaration.

#### 1B-2: Remove dead imports from `Flywheel.java`

- Remove `DynamicMotionMagicVoltage`, `MotionMagicVelocityVoltage`, `java.util.logging.Logger` — all imported, never used

#### 1B-3: Remove dead call `m_motor.getConfigurator()` from `Turret.java` (Issue L-1)

Return value is discarded, call does nothing. Remove it.

#### 1B-4: Fix double semicolon in `RunIntake.java` (Issue L-2)

```java
private double speed;;   // remove extra semicolon
```

#### 1B-5: Remove duplicate imports from `Vision.java` (Issue L-3)

`NetworkTableInstance` and `StructPublisher` are imported twice. Remove duplicates.

#### 1B-6: Fix `CoolSnurbo` missing `addRequirements()` (Issue H-5)

Without `addRequirements(flywheel)`, the scheduler is blind to this command's flywheel use.

```java
public CoolSnurbo(Flywheel m_flywheel) {
    this.m_flywheel = m_flywheel;
    addRequirements(m_flywheel);   // ADD
}
```

> **Note:** This causes `ShootingSequenceUTS`'s parallel group to have two commands requiring `Flywheel`. WPILib rejects this. Merge snurbo logic into `UpToSpeedHopperShoot`, or accept that `CoolSnurbo` is deleted in Stage 5 and defer this fix.

#### 1B-7: Resolve climber — restore or remove bindings (Issue H-6)

`setMotorSpeed()` has the motor call commented out. The climber never moves despite button bindings.

- **Option A (fix):** Uncomment the motor call with appropriate position safety checks
- **Option B (declare stub):** Remove `ClimbPole` button bindings from `RobotContainer` and add a visible comment

> **DIO pin conflict:** README.md lists DIO pin 4 for both the turret limit switch **and** the climber limit switch. Verify which device actually uses DIO 4 and assign unique pins before enabling the climber.

### 1C — Limelight Deprecation

> **Confirmed from README.md:** The robot has 4 cameras — `Thrifty_cam_1`, `Thrifty_cam_2` (PhotonVision), `limelight-gcd`, `limelight-gcc` (Limelight). The two Limelights are being removed.

#### 1C-1: Audit which cameras are actively wired

Map every constructor call in `RobotStateMachine.java` / `RobotContainer.java` to its camera implementation.

#### 1C-2: Replace `LimelightIO` instantiations with `PhotonVisionIO`

Use the same camera name and robot-to-camera transform. Remove heading/angular velocity supplier injections.

#### 1C-3: Delete `LimelightIO.java` and `LimelightHelpers.java`

- Delete `subsystems/vision/limelight/LimelightIO.java`
- Delete `subsystems/vision/limelight/LimelightHelpers.java`
- Delete empty `limelight/` package directory
- Remove any remaining `"limelight"` NT table name strings

### Stage 1 Verification ✓

1. Project compiles with zero errors
2. No references to `LimelightIO`, `LimelightHelpers`, `RangeFinder rangeFinder` instance, `predX`, `predY` remain
3. GCC and GCD pose publishers still visible in AdvantageScope (now from PhotonVision)
4. Git diff is deletions only

---

## Stage 2: Logging & Telemetry Improvements

**Risk: LOW — Additive changes — Can begin immediately after Stage 0**

Incorporates ISSUES.md items: **C-4, H-2, H-3, H-4, M-4, M-5**

### 2A — Capture console output in log files

#### 2A-1: Add `DataLogManager.logConsoleOutput()` to `Robot.java`

```java
DataLogManager.start();
DataLogManager.logConsoleOutput();   // ADD
```

### 2B — Rate-limit subsystem periodic telemetry (Issues C-4, H-4)

Rate-limiting is applied at the same time as key renaming so each file is touched once.

#### 2B-1: Rate-limit `AlignTurretToHub.execute()` (Issue C-4)

6 SmartDashboard writes fire every 20ms while shooting. The pose math and `turret.setPosition()` must remain every loop; only debug output is rate-limited.

```java
private final Timer m_debugTimer = new Timer();

@Override
public void execute() {
    turret.setPosition(clampedAngle);   // must run every loop

    if (m_debugTimer.advanceIfElapsed(0.1)) {
        SmartDashboard.putNumber("Turret/AlignmentErrorDeg", turretError);
        SmartDashboard.putNumber("Turret/DistToHubM", distance);
        SmartDashboard.putBoolean("Turret/IsAligned", isAligned);
        SmartDashboard.putNumber("Turret/PositionSetpointDeg", clampedAngle);
        SmartDashboard.putNumber("Turret/AlignErrorX", errorX);
        SmartDashboard.putNumber("Turret/AlignErrorY", errorY);
    }
}
```

### 2C — Consistent key naming (Subsystem/Key format)

#### 2C-1: Rename keys in `Flywheel.java` — and rate-limit (Issue H-4)

Add `private final Timer m_telemetryTimer = new Timer();` and wrap all puts at 10 Hz:

| Old Key | New Key |
|---------|---------|
| `"Left Motor Speed"` | `"Flywheel/VelocityRPS"` |
| `"Shot Multiplier"` | `"Flywheel/SpeedMultiplier"` |
| `"Rotation Multiplier"` | `"Flywheel/RotationMultiplier"` |
| `"Up to Speed"` | `"Flywheel/IsUpToSpeed"` |
| `"reqSpeed"` | `"Flywheel/ReqVelocityRPS"` |
| `"actSpeed"` | `"Flywheel/ActualVelocityRPS"` |
| `"isUnderTrench"` | `"Flywheel/IsUnderTrench"` |
| `"rot new testing"` | `"Turret/PositionConverted"` |
| `"rot adder"` | `"Turret/RotAdder"` |
| `"rot old testing"` | `"Turret/RobotRotDeg"` |
| `"flywheel initial speed"` | `"Flywheel/InitialSpeedRPS"` |
| `"flywheel sped-up speed"` | `"Flywheel/AdjustedSpeedRPS"` |

#### 2C-2: Rename keys in `Turret.java` — and rate-limit (Issue H-4)

| Old Key | New Key |
|---------|---------|
| `"switch on or off"` | `"Turret/LimitSwitch"` |
| `"Motor Position"` | `"Turret/MotorPositionRot"` |
| `"Turret Position"` | `"Turret/PositionDeg"` |
| `"Robot Rot in Deg"` | `"Turret/RobotHeadingDeg"` |
| `"UnconvPos"` | `"Turret/UnconvertedPos"` |

#### 2C-3: Rename keys in `AlignTurretToHub.java` — fix typos (rate-limited in 2B-1)

| Old Key | New Key |
|---------|---------|
| `"errorFromPrev.getX"` | `"Turret/AlignErrorX"` |
| `"errorFromPrev.getY"` | `"Turret/AlignErrorY"` |
| `"errorFromPrevRobotRot"` | `"Turret/AlignRobotRotError"` |
| `"errroFromPrevRot"` *(typo)* | `"Turret/AlignRotError"` |
| `"turretAndRobot"` | `"Turret/TurretPlusRobotAngle"` |
| `"Dist to Tag"` | `"Turret/DistToHubM"` |
| `"turretError"` | `"Turret/AlignmentErrorDeg"` |
| `"Aligned"` | `"Turret/IsAligned"` |
| `"tunring_pos_setpoint"` *(typo)* | `"Turret/PositionSetpointDeg"` |

#### 2C-4: Rename key in `Intake.java`

| Old Key | New Key |
|---------|---------|
| `"Deploy Pos"` | `"Intake/DeployPositionRot"` |

#### 2C-5: Rename `RobotStateMachine.java` keys (already rate-limited in Stage 0-2)

| Old Key | New Key |
|---------|---------|
| `"RobotState"` | `"Robot/State"` |
| `"FieldZone"` | `"Robot/FieldZone"` |
| `"Driver Connected"` | `"Robot/DriverConnected"` |
| `"Gunner Connected"` | `"Robot/GunnerConnected"` |
| `"Match Time"` | `"Robot/MatchTimeSec"` |
| `"distToTag2"` | `"Robot/DistToHubM"` |
| `"isFacing"` | `"Robot/IsFacingHub"` |
| `"VelX"` | `"Robot/FieldVelX"` |
| `"VelY"` | `"Robot/FieldVelY"` |
| `"ductTapeCorrections"` | `"Robot/DuctTapeCorrections"` |

### 2D — Add missing motor hardware telemetry

> **Note from README.md:** Swerve drive motors are REV SPARK MAX (IDs 1–8) — not TalonFX. Drive motor telemetry is already handled by `Telemetry.java`. Items below cover remaining TalonFX motors only.

#### 2D-1: `Flywheel.java` — add hardware entries (within rate-limited block)

```java
SmartDashboard.putNumber("Flywheel/Motor1StatorCurrentA", m_motor.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Flywheel/Motor1SupplyVoltageV", m_motor.getSupplyVoltage().getValueAsDouble());
SmartDashboard.putNumber("Flywheel/Motor1TempC",          m_motor.getDeviceTemp().getValueAsDouble());
SmartDashboard.putNumber("Flywheel/Motor2StatorCurrentA", m_motor2.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Flywheel/Motor2TempC",          m_motor2.getDeviceTemp().getValueAsDouble());
```

#### 2D-2: `Turret.java` — add hardware entries

```java
SmartDashboard.putNumber("Turret/StatorCurrentA", m_motor.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Turret/SupplyVoltageV", m_motor.getSupplyVoltage().getValueAsDouble());
SmartDashboard.putNumber("Turret/TempC",          m_motor.getDeviceTemp().getValueAsDouble());
```

#### 2D-3: `Intake.java` — add hardware entries

```java
SmartDashboard.putNumber("Intake/DeployStatorCurrentA", m_deployMotor.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Intake/DeploySupplyVoltageV", m_deployMotor.getSupplyVoltage().getValueAsDouble());
SmartDashboard.putNumber("Intake/RollerStatorCurrentA", m_rollerMotor.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Intake/RollerVelocityRPS",    m_rollerMotor.getVelocity().getValueAsDouble());
```

### 2E — Add telemetry to dark subsystems

#### 2E-1: Add `periodic()` to `Hopper.java` — currently has none

```java
@Override
public void periodic() {
    SmartDashboard.putNumber("Hopper/Motor1VelocityRPS",    m_motor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor1StatorCurrentA", m_motor1.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor2VelocityRPS",    m_motor2.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Hopper/Motor2StatorCurrentA", m_motor2.getStatorCurrent().getValueAsDouble());
}
```

#### 2E-2: Add logging to `Climber.java`

```java
SmartDashboard.putNumber("Climber/PositionRot",    m_motor.getPosition().getValueAsDouble());
SmartDashboard.putNumber("Climber/StatorCurrentA", m_motor.getStatorCurrent().getValueAsDouble());
SmartDashboard.putNumber("Climber/SupplyVoltageV", m_motor.getSupplyVoltage().getValueAsDouble());
SmartDashboard.putNumber("Climber/VelocityRPS",    m_motor.getVelocity().getValueAsDouble());
```

### 2F — System health

```java
SmartDashboard.putNumber("Robot/BatteryVoltageV",
    RobotController.getBatteryVoltage());
SmartDashboard.putNumber("Robot/CANBusUtilizationPct",
    RobotController.getCANStatus().percentBusUtilization * 100.0);
SmartDashboard.putBoolean("Robot/RSLStatus",
    RobotController.getRSLState());
```

Import: `edu.wpi.first.wpilibj.RobotController`

### 2G — Vision quality filtering (Issues H-2, H-3, M-4)

#### 2G-1: Cache `getVisionEst()` result — eliminate double-call (Issue H-2)

```java
// BEFORE — called twice per camera per loop:
for (VisionIO visionIO : m_visionOdometryCams) {
    visionIO.getVisionEst().ifPresent(est -> estimator.addVisionMeasurement(...));
    if (visionIO.getName().contains("gcc")) {
        visionIO.getVisionEst().ifPresent(est -> gccPub.set(...));
    }
}

// AFTER — cache result:
for (VisionIO visionIO : m_visionOdometryCams) {
    var result = visionIO.getVisionEst();
    result.ifPresent(est -> estimator.addVisionMeasurement(...));
    if (visionIO.getName().contains("gcc")) {
        result.ifPresent(est -> gccPub.set(est.getPose()));
    }
}
```

#### 2G-2: Add distance-based and ambiguity-based vision filtering (Issues H-3, M-4)

Current code blindly accepts all measurements with a static std deviation of 0.1m. A low-confidence single-tag detection can corrupt the pose estimate.

```java
visionIO.getVisionEst().ifPresent(est -> {
    // Reject if pose jumps more than 1m from current estimate
    double jump = est.getPose().getTranslation()
        .getDistance(estimator.getEstimatedPosition().getTranslation());
    if (jump > 1.0) return;

    // Scale trust based on distance from the detected tag
    double tagDist = est.targetsUsed().get(0)
        .getBestCameraToTarget().getTranslation().getNorm();
    Vector<N3> stdDev = VecBuilder.fill(
        0.1 * tagDist,
        0.1 * tagDist,
        Units.degreesToRadians(5 * tagDist));
    estimator.addVisionMeasurement(est.getPose(), est.getTimestamp(), stdDev);
});
```

#### 2G-3: Fix `UpToSpeedHopperShoot` SmartDashboard read in execute() (Issue M-5)

```java
private double m_targetSpeed;

@Override
public void initialize() {
    m_targetSpeed = SmartDashboard.getNumber("Shoot Speed",
        RangeFinder.getSpeed(stateMachine.getDistanceToHub()));
}

@Override
public void execute() {
    m_Flywheel.setSpeed(m_targetSpeed);   // no NT read in loop
}
```

### Stage 2 Verification ✓

1. Deploy, enable Teleop — open AdvantageScope via NT4
2. Confirm folders: `Flywheel/`, `Turret/`, `Hopper/`, `Intake/`, `Climber/`, `Robot/`
3. Motor current values non-zero and plausible while motors run
4. After match, open .wpilog — Console tab shows System.out messages
5. No old flat key names visible
6. Vision corrections: confirm pose doesn't jump when a single-tag detection fires far from the robot

---

## Stage 3: Shoot-on-the-Move Accuracy Fixes

**Risk: MEDIUM — Changes active aiming math — Requires hardware testing**

> The current TOF table values (0.85–1.14 s for 1.8–5.2 m) are labeled `// ! Fake values`. A ball at 15 m/s exit speed travels 1.8 m in ~0.12 s — not 0.85 s. Wrong TOF values make lead compensation worse than no compensation.

### 3A — Measure and replace fake TOF values

Film shots at 240 fps minimum. Measure at each distance in the current table. Average 3 shots per distance. Expected realistic values: ~0.10–0.30 s across the 1.8–5.2 m range.

Update `m_TOFMap` in `src/main/java/frc/robot/utility/RangeFinder.java` and remove `// ! Fake values`.

### 3B — Wire turret-angle flywheel speed correction

The formula exists in `RangeFinder.getRotAdder(deg)` but is not wired into the speed setpoint.

```java
double baseShotVelocity  = RangeFinder.getShotVelocity(distance);
double turretAngleDeg    = stateMachine.getConvertedTurretPosition();
double correctedVelocity = baseShotVelocity + RangeFinder.getRotAdder(turretAngleDeg);
m_Flywheel.setSpeed(correctedVelocity);
```

> Validate the +2.83 RPS constant term at zero turret offset. If the measurement table already accounts for it, zero the constant to avoid double-applying.

### 3C — Fix Bug 2: radial speed correction unit issue

The TOF-iterated loop already computes the corrected distance — `RangeFinder.getShotVelocity(correctedDistance)` inherently accounts for the longer/shorter shot. Remove any additional radial speed correction term that adds robot velocity (m/s) directly to flywheel RPS. Confirm `UpToSpeedHopperShoot` uses `stateMachine.getTargetPose()` distance, not raw `HubPose` distance, for its speed lookup.

### Stage 3 Verification ✓

1. Stationary shots accurate at each measured distance
2. Lateral movement at ~1.5 m/s: shot accuracy maintained
3. Radial movement toward hub at ~1.5 m/s: shot no longer overshoots
4. `Flywheel/AdjustedSpeedRPS` shows a non-trivial adder when turret is off-center

---

## Stage 4: Hackbots Integration — Files & Compile

**Risk: MEDIUM — Copy and simplify only — No existing code modified yet**

> **Hackbots source:** GitHub — https://github.com/hackbots-3414/2026_Rebuilt  
> Local clone: `C:\projects\Gearcats\Hackbots\2026_Rebuilt\`

### 4-1: Create `frc.robot.aiming` package — copy 7 files from Hackbots

Source: `C:\projects\Gearcats\Hackbots\2026_Rebuilt\CompBot\aiming\`

| File | Notes |
|------|-------|
| `AimParams.java` | Data record — no changes |
| `AimStrategy.java` | Interface — no changes |
| `AimConstraints.java` | Angle limits — values set in Stage 8 |
| `ToFAim.java` | Primary strategy — 15-iteration TOF convergence |
| `PhysicsAim.java` | Ballistics solver |
| `TuneAim.java` | Manual tuning strategy |
| `AimMeasurement.java` | Measurement data class |

Update all `package` declarations to `frc.robot.aiming`.

### 4-2: Create `frc.robot.subsystems.shooter` package — copy 5 files from Hackbots

Source: `C:\projects\Gearcats\Hackbots\2026_Rebuilt\CompBot\subsystems\shooter\`

| File | Notes |
|------|-------|
| `Shooter.java` | Flywheel + hood control |
| `ShooterConstants.java` | **Edit:** All CAN IDs = 0 placeholder; measurement table re-tuned in Stage 8 |
| `ShooterIO.java` | Interface — no changes |
| `ShooterIOHardware.java` | CAN IDs from ShooterConstants |
| `ShooterIOSim.java` | Simulation — no changes |

### 4-3: Replace `subsystems/turret/` contents

Source: `C:\projects\Gearcats\Hackbots\2026_Rebuilt\CompBot\subsystems\turret\`

| File | Action | Key Changes |
|------|--------|-------------|
| `Turret.java` | Copy as-is | DynamicMotionMagic + `findCC()` kept |
| `TurretConstants.java` | Copy + Edit | CAN IDs = 0; **remove** `kEncoder2Id` |
| `TurretIO.java` | Copy as-is | Interface |
| `TurretIOHardware.java` | Copy + Major Edit | Remove second CANcoder + all CRT math (see 4-4) |
| `TurretIOSim.java` | Copy as-is | No changes |

### 4-4: Simplify `TurretIOHardware.java` — single absolute encoder

Hackbots uses two CANcoders + CRT math. Gearcats uses one magnetic absolute encoder.

**Remove:** `kEncoder2Id`, second `CANcoder`, all CRT calculation, encoder cross-validation logic

**Keep/Simplify to:** Single `CANcoder` in absolute mode; TalonFX feedback = `RemoteCANcoder`; encoder value IS the turret angle

### 4-5: Copy commands — `AimPrep.java` and `ShootWhenReady.java`

Copy into `frc.robot.commands`. Update package declarations. Leave `StateManager` references as stubs — wired in Stage 5.

### 4-6: Add placeholder CAN IDs to `Constants.java`

```java
// TBD at wiring — was: Turret=12, Flywheel R=13, L=14
public static final int kTurretYawMotorID    = 0;
public static final int kTurretCANcoderID    = 0;
public static final int kShooterMotorRightID = 0;
public static final int kShooterMotorLeftID  = 0;
public static final int kHoodMotorID         = 0;
public static final int kHoodCANcoderID      = 0;

// TBD from CAD/measurement
public static final Transform2d ROBOT_TO_TURRET_BASE = new Transform2d(
    new Translation2d(0.0, 0.0), new Rotation2d());  // PLACEHOLDER
```

### 4-7: Get project to compile with zero errors

Resolve all import errors. Old Gearcats files remain untouched until Stage 5.

### Stage 4 Verification ✓

1. Zero compile errors
2. Existing robot behavior unchanged
3. `TurretIOHardware` has no CRT logic, no second CANcoder
4. All placeholder CAN IDs are 0 in Constants

---

## Stage 5: Hackbots Integration — State Machine Wire-Up

**Risk: HIGH — Replaces core shooting subsystems — Largest single change**

> **Architecture decision:** Keep the Gearcats `RobotStateMachine` singleton. Adapt Hackbots StateManager logic into `RobotStateMachine` methods — less disruptive than adopting the full Superstructure pattern.

### 5-1: Replace `Flywheel` with `Shooter`

| Old | New |
|-----|-----|
| `Flywheel m_Flywheel` | `Shooter m_Shooter` |
| `new Flywheel(this)` | `new Shooter(new ShooterIOHardware())` |
| `getFlywheel()` | `getShooter()` |

### 5-2: Replace `Turret` constructor to use `TurretIOHardware`

| Old | New |
|-----|-----|
| `new Turret(this)` | `new Turret(new TurretIOHardware())` |
| `getConvertedTurretPosition()` | `getAimParams().yaw` |

### 5-3: Add `getAimParams()`, `isShootReady()`, `ShootMode`

```java
public AimParams getAimParams() {
    return m_aimStrategy.update(HubPose, getTurretPose(), getFieldSpeeds().toTranslation2d());
}

public boolean isShootReady() {
    AimParams params = getAimParams();
    return params.status() != AimStatus.Impossible
        && m_Turret.isOnTarget()
        && m_Shooter.tracked()
        && hasValidOdometry();
}

public enum ShootMode { Scoring, Feeding, Donut }
```

### 5-4: Remove automatic flywheel speed control from `periodic()`

Shooter speed is now owned entirely by `Shooter` internally.

### 5-5: Remove `updateTargetPose()`

Superseded by `ToFAim`'s built-in 15-iteration loop.

### 5-6: Delete old files now superseded

| File to Delete | Replaced By |
|----------------|-------------|
| `subsystems/turret/Flywheel.java` | `subsystems/shooter/Shooter.java` |
| `utility/RangeFinder.java` | `aiming/` package |
| `commands/CoolSnurbo.java` | Speed reduction internal to Shooter |
| `commands/UpToSpeedHopperShoot.java` | `commands/ShootWhenReady.java` |
| `commands/ShootingSequence.java` | `commands/AimPrep.java` |
| `commands/ShootingSequenceUTS.java` | `commands/AimPrep.java` |
| `commands/ShootFuel.java` | `AimPrep` + `ShootWhenReady` |
| `commands/AlignTurretToHub.java` | Turret tracking inside `AimPrep` |
| `commands/SetTurretAngle.java` | `Turret.home()` / `Turret.forwards()` |

**Keep:** `commands/MoveTurret.java`

### Stage 5 Verification ✓

1. Zero compile errors after deletion
2. `RobotStateMachine` owns `Shooter` and Hackbots `Turret`
3. `getAimParams()`, `isShootReady()`, `ShootMode` all present
4. No references to deleted files remain

---

## Stage 6: Hackbots Integration — RobotContainer & Autonomous

**Risk: HIGH — Changes controller bindings and auto paths**

### 6-1: Update field references and controller bindings

| Button / Axis | Old Command | New Command |
|---------------|-------------|-------------|
| Gunner Left Trigger | `ShootingSequenceUTS` | `AimPrep` + `ShootWhenReady` |
| Gunner Left Bumper | `ShootingSequenceUTS` (no align) | `AimPrep` only |
| Gunner Start | `Turret.zeroMotorPosition()` | `Turret.home()` |
| Gunner X | `Turret.goToZero()` | `Turret.home()` |
| Driver Right Bumper | `CoolSnurbo` | Remove |
| Drive speed axes | Multiplied by `m_flywheel.speedModifier` | Remove multiplier |

### 6-2: Update NamedCommands — preserve string names so .auto files don't need editing

| Old NamedCommand String | New Registration |
|-------------------------|-----------------|
| `"ShootFuel"`, `"NewShootFuel"`, `"ManualShootFuel*"` | `AimPrep` + `ShootWhenReady` |
| `"AlignTurret"`, `"TrenchStartAngle"` | `AimPrep` (tracking-only) |
| `"ShootingSequence*"` | `AimPrep` + `ShootWhenReady` |

### 6-3: Apply Stage 2 logging conventions to new subsystems

- `"Shooter/HoodAngleDeg"`, `"Shooter/VelocityRPS"`, `"Shooter/IsTracked"`
- `"Aiming/Status"`, `"Aiming/TargetYawDeg"`, `"Aiming/TargetPitchDeg"`

### Stage 6 Verification ✓

1. Robot enables in Teleop — no crashes
2. Gunner left trigger: turret tracks hub, hood adjusts with distance
3. `ShootWhenReady` releases hopper only when `Robot/IsShootReady` is true
4. Manual turret override (POV left/right) still works
5. At least one PathPlanner auto path runs without exception
6. Drive speed NOT reduced during shooting

---

## Stage 7: Lead Compensation Enhancement

**Risk: MEDIUM — New capability — Depends on Stage 6 verified + Stage 3 TOF data**

> **Code comparison finding:** archive2025 scored 82% SOTM capability vs 2026_Rebuilt at 28%. This stage closes that gap using archive2025's iterative lead logic combined with Hackbots' physics-grounded TOF.

### 7-1: Create `LeadCompensator.java` in `frc.robot.aiming`

```java
public class LeadCompensator {
    public static Pose2d computeLeadPose(
            Pose2d hubPose, Pose2d turretPose,
            Translation2d fieldVelocity,
            AimStrategy aimStrategy, AimConstraints constraints) {

        Pose2d best = hubPose;
        for (int i = 0; i < 5; i++) {
            AimParams params = aimStrategy.update(best, turretPose, fieldVelocity);
            double tof = params.tof();
            best = new Pose2d(
                hubPose.getX() - (fieldVelocity.getX() * tof),
                hubPose.getY() - (fieldVelocity.getY() * tof),
                new Rotation2d());
        }
        return best;
    }
}
```

5 iterations vs archive2025's 20 — converges in 3–4 at any realistic FRC speed. TOF from physics solver, not lookup table.

### 7-2: Wire into `RobotStateMachine.getAimParams()`

```java
public AimParams getAimParams() {
    Pose2d leadTarget = LeadCompensator.computeLeadPose(
        HubPose, getTurretPose(),
        getFieldSpeeds().toTranslation2d(),
        m_aimStrategy, m_aimConstraints);
    return m_aimStrategy.update(leadTarget, getTurretPose(), getFieldSpeeds().toTranslation2d());
}
```

### 7-3: Add telemetry and turret-angle rotation correction

```java
SmartDashboard.putNumber("Aiming/LeadOffsetX", leadTarget.getX() - HubPose.getX());
SmartDashboard.putNumber("Aiming/LeadOffsetY", leadTarget.getY() - HubPose.getY());
SmartDashboard.putNumber("Aiming/TOFSec", latestParams.tof());

// Rotation correction in ShootWhenReady or Shooter:
double correctedRPS = params.speed()
    + (0.000725312 * Math.pow(Math.abs(params.yaw()), 2) + 2.82988);
m_Shooter.setFlywheelSpeed(correctedRPS);
```

### Stage 7 Verification ✓

1. `Aiming/LeadOffsetX/Y` non-zero when moving, zero when stationary
2. Lateral shot at 2 m/s: hits hub
3. Radial toward hub at 2 m/s: hits hub
4. Radial away from hub at 2 m/s: hits hub
5. Stationary shots unchanged

---

## Stage 8: Hardware Wiring, Tuning & Measurement

**Risk: HARDWARE — Blocked by physical build**

| # | Item | How to Complete |
|---|------|-----------------|
| 8-1 | **Assign CAN IDs** — all new turret/shooter motors and encoders | Update all placeholder 0 values. Reference existing: Turret=12, Flywheel R=13 L=14, Hopper=22/23, Intake=20/21, Climber=25, CANdle=50 |
| 8-2 | **Turret mounting offset** — `ROBOT_TO_TURRET_BASE` | Measure from CAD or tape measure on assembled robot |
| 8-3 | **Absolute encoder zero alignment** | Place magnet so encoder reads 0.0 when turret faces forward. Verify with Tuner X |
| 8-4 | **Confirm turret travel ≤ ±180°** | Drive to each soft-limit stop physically. If >360° total, single absolute encoder is insufficient |
| 8-5 | **Set hood angle range** — `AimConstraints` min/max | Drive hood to physical limits; record encoder values at each end |
| 8-6 | **Re-characterize PID and MotionMagic gains** | Use SysID or Tuner X live tuning. Hackbots values are starting points only |
| 8-7 | **Fix Turret PID Slot 1 defaults to zero (Issue M-1)** | `slot1Configs.kP = SmartDashboard.getNumber("kP", TurretConstants.kP);` |
| 8-8 | **Resolve DIO pin conflict** | README lists DIO 4 for both turret limit switch and climber limit switch — assign unique pins |
| 8-9 | **Measure real TOF values** (if not done in Stage 3) | 240 fps+ video at 6–12 distances from 1.5–6.5 m |
| 8-10 | **Build measurement table** | Use `TuneAim` strategy. Record 8–12 points, then switch to `ToFAim` to validate interpolation |
| 8-11 | **Validate lead compensation** | Drive at known speed in all directions. Confirm hits with logged `Aiming/LeadOffset` values |

---

## Appendix A — ISSUES.md Resolution Map

| ID | Severity | Issue | Stage | Resolution |
|----|----------|-------|-------|------------|
| C-1 | Critical | `System.gc()` every 100ms | 0-1 | Deleted |
| C-2 | Critical | 10+ SmartDashboard writes every loop in StateMachine | 0-2 | Rate-limited 10 Hz |
| C-3 | Critical | New `Pose2d` objects every loop | 0-2 | `updateTargetPose()` at 10 Hz |
| C-4 | Critical | 6 SmartDashboard writes in `AlignTurretToHub.execute()` | 2B-1 | Rate-limited 10 Hz |
| H-1 | High | 4 CAN reads in `getChassisSpeeds()` | 0-3 | Cached in drivetrain |
| H-2 | High | `getVisionEst()` called twice per camera | 2G-1 | Cached in local variable |
| H-3 | High | No vision quality filtering | 2G-2 | Distance + ambiguity filtering |
| H-4 | High | SmartDashboard writes every loop in Turret/Flywheel | 2C | Rate-limited 10 Hz with rename |
| H-5 | High | `CoolSnurbo` missing `addRequirements()` | 1B-6 | Fixed or superseded Stage 5 |
| H-6 | High | Climber motor permanently commented out | 1B-7 | Fix or remove bindings |
| H-7 | High | Non-thread-safe singleton | 0-4 | Eager initialization |
| M-1 | Medium | Turret PID Slot 1 gains default to zero | 8-7 | Add constant fallback |
| M-2 | Medium | `setPosition()` no bounds check | 0-5 | `MathUtil.clamp()` added |
| M-3 | Medium | `RobotConfig` null if load fails | 0-6 | Null safety + fallback |
| M-4 | Medium | Static vision std devs trust all equally | 2G-2 | Dynamic distance-scaled std devs |
| M-5 | Medium | SmartDashboard read every `execute()` | 2G-3 | Read once in `initialize()` |
| L-1 | Low | Dead `m_motor.getConfigurator()` | 1B-3 | Removed |
| L-2 | Low | Double semicolon in `RunIntake.java` | 1B-4 | Removed |
| L-3 | Low | Duplicate imports in `Vision.java` | 1B-5 | Removed |
| L-4 | Low | `kS` TODO — static friction | 8-6 | Verify `kS=0.3087` adequate at tuning |
| L-5 | Low | Intake deploy no soft limit | 8 | Add TalonFX soft limit or timer cutoff |

---

## Appendix B — File Change Summary

| File | Stages | Change Type |
|------|--------|-------------|
| `Robot.java` | 0-1, 2A, 2F | Delete System.gc(); add logConsoleOutput(); add system health |
| `RobotStateMachine.java` | 0-2, 0-3, 0-4, 1A, 2C-5, 5 | Rate-limit; cache speeds; eager singleton; fix nextPose; rename keys; replace Flywheel with Shooter |
| `Constants.java` | 0-6, 4-6 | RobotConfig null safety; add placeholder CAN IDs |
| `CommandSwerveDrivetrain.java` | 0-3 | Cache ChassisSpeeds |
| `Turret.java` (old) | 0-5, 1B-3, 2C-2, 2D-2 | Bounds clamp; remove dead call; rename + rate-limit; hardware telemetry → **deleted Stage 5** |
| `Flywheel.java` | 1B-2, 2C-1, 2D-1 | Remove dead imports; rename + rate-limit; hardware telemetry → **deleted Stage 5** |
| `AlignTurretToHub.java` | 2B-1, 2C-3 | Rate-limit execute(); rename keys + fix typos → **deleted Stage 5** |
| `Intake.java` | 2C-4, 2D-3 | Rename key; add hardware telemetry |
| `Hopper.java` | 2E-1 | Add periodic() |
| `Climber.java` | 1B-7, 2E-2 | Fix or remove; add telemetry |
| `Vision.java` | 1B-5, 2G | Remove duplicate imports; cache result; add filtering |
| `UpToSpeedHopperShoot.java` | 2G-3 | Read speed once in initialize() → **deleted Stage 5** |
| `RunIntake.java` | 1B-4 | Fix double semicolon |
| `RobotContainer.java` | 1B-1, 6 | Remove dead RangeFinder; update bindings and NamedCommands |
| `RangeFinder.java` | 3A | Replace fake TOF values → **deleted Stage 5** |
| `LimelightIO.java` | 1C | **Deleted** — replaced by PhotonVisionIO |
| `LimelightHelpers.java` | 1C | **Deleted** — vendor library no longer needed |
| `CoolSnurbo.java` | 1B-6 | Fix addRequirements → **deleted Stage 5** |
| `aiming/*.java` (7 files) | 4-1 | **New** — copied from Hackbots |
| `aiming/LeadCompensator.java` | 7-1 | **New** — ported from archive2025 logic |
| `subsystems/shooter/*.java` (5 files) | 4-2 | **New** — copied from Hackbots |
| `subsystems/turret/*.java` (5 files) | 4-3, 4-4 | **Replaced** — Hackbots versions; TurretIOHardware simplified to single encoder |
| `commands/AimPrep.java` | 4-5 | **New** — copied from Hackbots |
| `commands/ShootWhenReady.java` | 4-5 | **New** — copied from Hackbots |

---

## Appendix C — Hardware Reference (from README.md)

| CAN ID | Device | Role | Status |
|--------|--------|------|--------|
| 1–4 | SPARK MAX | Swerve drive | Active |
| 5–8 | SPARK MAX | Swerve steer | Active |
| 12 | TalonFX | Turret yaw | Replaced Stage 4 |
| 13 | TalonFX | Flywheel right | Replaced Stage 4 |
| 14 | TalonFX | Flywheel left | Replaced Stage 4 |
| 20 | TalonFX | Intake roller | Active |
| 21 | TalonFX | Intake deploy | Active |
| 22 | TalonFX | Hopper indexer | Active |
| 23 | TalonFX | Hopper kicker | Active |
| 25 | TalonFX | Climber | Disabled (H-6) |
| 50 | CANdle | LEDs — 177 strip + 8 onboard | Active |
| TBD | TalonFX | Turret yaw (new) | Stage 4 |
| TBD | CANcoder | Turret absolute encoder (single) | Stage 4 |
| TBD | TalonFX×2 | Flywheel left/right (new) | Stage 4 |
| TBD | TalonFX | Hood motor (new) | Stage 4 |
| TBD | CANcoder | Hood encoder (new) | Stage 4 |

**DIO Pin 4** — Turret home limit switch *(conflict: README also lists DIO 4 for Climber — resolve at Stage 8-8)*

---

*Gearcats FRC Team 6500 — Rebuilt2026 Master Development Plan — Branch: leto — Updated 2026-09-05*

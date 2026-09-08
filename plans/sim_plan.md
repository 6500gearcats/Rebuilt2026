# Rebuilt2026 — Simulation Improvement Plan

Covers pre-build simulation work to validate robot logic before hardware is available.  
Track execution in `SIM_PROGRESS.md`.

**Branch:** `leto`  
**Goal:** Make the WPILib desktop simulator useful enough to catch timing, sequencing, and  
control-loop issues before the first practice day.

---

## What Already Works in Sim (No Changes Needed)

| Area | Status |
|------|--------|
| Swerve drivetrain | CTRE `updateSimState()` wired — odometry and PathPlanner paths run |
| Turret | `TurretIOSim` has first-order lag dynamics (80/20 filter) |
| PhotonVision cameras | `PhotonVisionSimIO` generates synthetic AprilTag detections from simulated pose |
| Pose estimation | Kalman filter runs with synthetic vision data |
| PathPlanner autos | All 50 paths run; named commands fire at correct waypoints |
| RobotStateMachine | State transitions, field zone logic, alliance detection all active |
| Controller bindings | Desktop joystick emulator allows button-press testing |

**How to launch:** VS Code WPILib extension → "Simulate Robot Code", then open  
AdvantageScope and connect to **localhost**.

---

## S-1 — Remove Limelight Dead Code

Limelight is not on the new robot. `LimelightIO` and `LimelightHelpers` were instantiated  
nowhere after the old robot's code was refactored. `VisionEstimate` had a Limelight  
constructor that was also dead.

**Files changed:**
- Delete `src/main/java/frc/robot/subsystems/vision/limelight/LimelightIO.java`
- Delete `src/main/java/frc/robot/subsystems/vision/limelight/LimelightHelpers.java`
- Rewrite `VisionEstimate.java` — remove Limelight import, field, and constructor; keep PhotonVision path only

| Task | Description |
|------|-------------|
| S1-1 | Grep confirms zero external callers of `LimelightIO` |
| S1-2 | Delete `limelight/` package directory |
| S1-3 | Strip `VisionEstimate.java` to PhotonVision-only |
| S1-4 | Compile verify |

---

## S-2 — Fix PhotonVision API Deprecations

Six `[removal]` warnings remain across `PhotonVisionIO.java` and `PhotonVisionSimIO.java`.  
These are the only remaining VS Code problems. Fixing them also makes the sim camera  
implementation use the current API, which matters because `getLatestResult()` is gone  
in newer PhotonLib versions.

**Deprecated API → replacement:**

| Old | New |
|-----|-----|
| `new PhotonPoseEstimator(layout, strategy, transform)` | `new PhotonPoseEstimator(layout, strategy, camera, transform)` |
| `camera.getLatestResult()` | `camera.getAllUnreadResults()` — returns `List<PhotonPipelineResult>` |
| `estimator.update(PhotonPipelineResult)` | `estimator.update(PhotonPipelineResult)` → `estimator.update()` (no arg) |

**Files:**
- `src/main/java/frc/robot/subsystems/vision/photonvision/PhotonVisionIO.java`
- `src/main/java/frc/robot/subsystems/vision/photonvision/PhotonVisionSimIO.java`

**Why it matters for sim:** `getAllUnreadResults()` returns every frame since the last poll  
rather than just the most recent — this prevents missed detections when the sim camera  
runs faster than the robot loop.

| Task | Description |
|------|-------------|
| S2-1 | Read both files and map all 6 deprecation sites |
| S2-2 | Update `PhotonPoseEstimator` constructor calls (2 sites) |
| S2-3 | Replace `getLatestResult()` with `getAllUnreadResults()` loop (3 sites) |
| S2-4 | Replace deprecated `estimator.update(result)` (2 sites) |
| S2-5 | Compile — confirm zero warnings remain |

---

## S-3 — ShooterIOSim: Add Flywheel Ramp Dynamics

`ShooterIOSim` currently snaps instantly to any setpoint, so `IsUpToSpeed` is always  
true the moment a velocity is commanded. This means the "wait for flywheel" gate in  
`ShootWhenReady` never actually blocks — all timing tests in sim pass trivially.

**Fix:** Replace the instant-snap with a first-order lag that mimics realistic ramp-up  
(~0.5–1.0 s to reach full speed). No physics library needed — same pattern as  
`TurretIOSim` (80/20 exponential filter each loop cycle).

```java
// Each updateInputs() call (50 Hz):
shooterVelocity = shooterVelocity.times(0.92).plus(target.times(0.08));
// 0.92/0.08 gives ~1.1 s to reach 95% — tune to match real motor if known
```

Also add a ramp-**down** so stopping the flywheel mid-auto doesn't look instantaneous.

**File:** `src/main/java/frc/robot/subsystems/shooter/ShooterIOSim.java`

| Task | Description |
|------|-------------|
| S3-1 | Add `targetVelocity` field; change `setVelocity()` to set target only |
| S3-2 | Add exponential filter to `updateInputs()` for both velocity fields |
| S3-3 | Verify `Shooter.isUpToSpeed()` threshold logic works with the lag |
| S3-4 | Run an auto in sim and confirm the shot waits for spin-up |

---

## S-4 — Hopper: Add CTRE SimState Feedback

`Hopper` drives two TalonFX motors with `motor.set(speed)` but reads nothing back  
(`periodic()` calls `getVelocity()` which returns 0 in sim because CTRE sim state  
is not seeded). Result: Hopper telemetry shows all zeros during sim runs.

**Fix:** In each periodic call, write a synthetic velocity back through `getSimState()`  
proportional to the last commanded duty cycle. This makes Hopper/IndexerVelocityRPS  
and Hopper/KickerVelocityRPS show realistic values in AdvantageScope during auto runs.

```java
// In Hopper.periodic(), after the SmartDashboard puts:
if (RobotBase.isSimulation()) {
    m_hopperMotor.getSimState().setRotorVelocity(
        m_hopperMotor.get() * kFreeSpeedRPS);
    m_kickerMotor.getSimState().setRotorVelocity(
        m_kickerMotor.get() * kFreeSpeedRPS);
}
```

**File:** `src/main/java/frc/robot/subsystems/hopper/Hopper.java`

| Task | Description |
|------|-------------|
| S4-1 | Add `RobotBase.isSimulation()` guard + `getSimState().setRotorVelocity()` calls |
| S4-2 | Define `kFreeSpeedRPS` constant (Falcon 500 free speed ≈ 100 RPS) |
| S4-3 | Confirm Hopper telemetry shows non-zero values in sim |

---

## S-5 — Intake: Add CTRE SimState Feedback

Same problem as Hopper. `Intake` uses TalonFX motors directly with no sim seeding.  
`Intake/DeployPositionRot` and roller velocity show zero in sim.

**Fix:** Seed position and velocity via `getSimState()` for both deploy and roller motors.  
The deploy motor is position-controlled, so integrate velocity → position each loop.

**File:** `src/main/java/frc/robot/subsystems/intake/Intake.java`

| Task | Description |
|------|-------------|
| S5-1 | Read `Intake.java` and identify motor field names and control mode |
| S5-2 | Add simState seeding for roller motor (velocity) |
| S5-3 | Add simState seeding for deploy motor (velocity + position integration) |
| S5-4 | Confirm Intake telemetry shows realistic values in sim |

---

## S-6 — Auto Path Validation Procedure

Document a repeatable checklist for running auto paths in sim and validating them  
before the robot is assembled. This is the highest-value thing sim enables right now.

**Checklist (to encode in SIM_PROGRESS.md once done):**

1. Launch sim: VS Code → "Simulate Robot Code"
2. Open AdvantageScope → connect to localhost
3. Open Field2d widget → set to `"Field"` NT key
4. Enable robot in auto mode
5. Select an auto path from the SmartDashboard chooser
6. Observe:
   - Robot follows path shape correctly (no path jumps)
   - Named commands fire at the right waypoints (check AdvantageScope log)
   - Turret tracking engages and disengages at correct moments
   - `ShootFuel*s` timeouts actually end the command sequence
   - Robot ends in the expected field position

**Paths to prioritize** (ones likely used in competition):
- All `LYahoo`, `PHM`, `FinalHope` paths — multi-ball autos
- `WakeCenter1`, `WakeCenter2` — center start variants

| Task | Description |
|------|-------------|
| S6-1 | Run all named auto paths once in sim, note any that crash or deviate badly |
| S6-2 | Verify `ShootFuel3s` through `ShootFuel10s` timeout correctly |
| S6-3 | Confirm `AlignTurret` named command engages in sim (turret moves) |
| S6-4 | Document any path-specific issues found |

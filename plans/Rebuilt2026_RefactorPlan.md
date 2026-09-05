# Gearcats Rebuilt2026 — Master Development Plan

**Branch:** `leto`  
**Working directory:** `c:\projects\Gearcats\Rebuilt2026\Rebuilt2026`  
**Updated:** 2026-09-05  
**Covers:** Loop overrun fixes · Dead code · Limelight deprecation · Logging · Pre-integration cleanup · Hackbots integration · Lead compensation

---

## Plan Overview

Nine ordered stages. Stages 0–3 fix and improve the existing Rebuilt2026 code. Stages 4–6 are the Hackbots integration. Stages 7–8 are enhancements and hardware. **Stage 0 must be done before anything else** — the robot is currently producing loop overrun warnings that will cause jerky driving and unreliable shooting.

| Stage | Name | Risk | Status | Dependency |
|-------|------|------|--------|------------|
| **0** | **Critical Performance Fixes (Loop Overruns)** | **LOW** | ✅ Done | None |
| 1 | Dead Code Removal & Limelight Deprecation | LOW | ✅ Done | Stage 0 |
| 2 | Logging & Telemetry Improvements | LOW | ✅ Done | Stage 0 |
| 3 | Pre-Integration Cleanup | LOW | ✅ Done | Stages 1–2 |
| 4 | Hackbots Integration — Files & Compile | MEDIUM | ✅ Done | Stage 3 |
| 5 | Hackbots Integration — State Machine Wire-Up | HIGH | ✅ Done | Stage 4 |
| 6 | Hackbots Integration — RobotContainer & Auto | HIGH | ⬜ Next | Stage 5 |
| 7 | Lead Compensation Enhancement | MEDIUM | ⬜ | Stage 6 verified |
| 8 | Hardware Wiring, Tuning & Measurement | HARDWARE | ⬜ | Stages 4–7 |

> **Source of Stage 0 issues:** The robot was showing WPILib loop overrun warnings (`Loop time of 0.025s is greater than the expected 0.02s`). Stages 0–2 addressed all 20 issues from ISSUES.md. Stage 3 closed the remaining dead-code and climber issues before the Hackbots integration began.

---

## Stage 0: Critical Performance Fixes (Loop Overruns) ✅

**Risk: LOW — Rate-limiting and one-line deletions only — No control logic changes**

The WPILib scheduler runs every 20ms. If any single loop cycle takes more than 20ms total (across all `periodic()` calls and running commands), WPILib prints a warning to the Driver Station console and the next cycle starts late. Sustained overruns cause jerky driving, missed vision frames, and unreliable shooting.

### ISSUES.md Cross-Reference

| ID | Issue | Fix Location |
|----|-------|-------------|
| C-1 | `System.gc()` every 100ms — guarantees GC pauses of 5–50ms | `Robot.java` |
| C-2 | 10+ SmartDashboard writes every loop in `RobotStateMachine.periodic()` | `RobotStateMachine.java` |
| C-3 | New `Pose2d`/`Rotation2d` objects created every loop — GC pressure | `RobotStateMachine.java` |
| H-1 | 4 CAN reads in `getChassisSpeeds()` called every loop | `RobotStateMachine.java` |
| H-7 | Non-thread-safe singleton — race condition at startup | `RobotStateMachine.java` |
| M-2 | `setPosition()` sends arbitrary angles without bounds check | `Turret.java` |
| M-3 | `RobotConfig` remains `null` if GUI settings file missing | `Constants.java` |

### Items Completed

| ID | Description | File | Commit |
|----|-------------|------|--------|
| C-1 | Delete `System.gc()` + `m_gcTimer` | `Robot.java` | stage-0 |
| C-2 | Rate-limit 10+ SmartDashboard writes in `RobotStateMachine.periodic()` | `RobotStateMachine.java` | stage-0 |
| C-3 | Rate-limit `Pose2d`/`Rotation2d` allocations | `RobotStateMachine.java` | stage-0 |
| H-1 | Cache `ChassisSpeeds` — replace 4 module-state reads with `getState().Speeds` | `RobotStateMachine.java` | stage-0 |
| H-7 | Fix non-thread-safe singleton — eager init | `RobotStateMachine.java` | stage-0 |
| M-2 | Add `MathUtil.clamp()` bounds check to `Turret.setPosition()` | `Turret.java` | stage-0 |
| M-3 | Improve `RobotConfig` null-safety error message | `Constants.java` | stage-0 |

---

## Stage 1: Dead Code Removal & Limelight Deprecation ✅

**Risk: LOW — Purely subtractive — No functional changes**

### Items Completed

| ID | Description | File | Commit |
|----|-------------|------|--------|
| L-1 | Remove dead `getConfigurator()` call | `Turret.java` | stage-1 |
| L-2 | Remove double semicolon | `RunIntake.java` | stage-1 |
| L-3 | Remove duplicate imports | `Vision.java` | stage-1 |
| H-5 | Add `addRequirements(m_flywheel)` to `CoolSnurbo` | `CoolSnurbo.java` | stage-1 |
| H-6 | Climber motor permanently commented out — declared stub, bindings kept | `Climber.java` | stage-1 |
| — | Deprecate all Limelight code (`LimelightIO.java`, `LimelightHelpers.java`) | Multiple | stage-1 |
| — | Remove dead Limelight references from `RobotContainer.java` | `RobotContainer.java` | stage-1 |

> **Note — 1B-1 missed:** `private final RangeFinder rangeFinder = new RangeFinder()` at `RobotContainer.java` was not removed in Stage 1. Addressed in Stage 3.

---

## Stage 2: Logging & Telemetry Improvements ✅

**Risk: LOW — Additive changes**

### Items Completed

| ID | Description | File | Commit |
|----|-------------|------|--------|
| — | Add `DataLogManager.logConsoleOutput(true)` | `Robot.java` | stage-2 |
| — | Add system health (battery V, CAN %, RSL) at 10 Hz | `Robot.java` | stage-2 |
| — | Rename SmartDashboard keys to `"Subsystem/Key"` format | Multiple | stage-2 |
| — | Add motor hardware telemetry (current, voltage, temp) to Flywheel/Turret | Multiple | stage-2 |
| — | Add `Hopper.periodic()` with motor telemetry | `Hopper.java` | stage-2 |
| — | Add Intake motor hardware telemetry | `Intake.java` | stage-2 |
| H-2 | Fix `getVisionEst()` called twice per loop — cache in local variable | `Vision.java` | stage-2 |
| H-3 | Add vision measurement filtering (reject >4 m from odometry estimate) | `Vision.java` | stage-2 |
| H-4 | Rate-limit Turret/Flywheel periodic SmartDashboard writes at 10 Hz | Multiple | stage-2 |
| M-4 | Scale vision std devs linearly with distance via `addVisionMeasurement` overload | `Vision.java` | stage-2 |
| M-5 | Cache `SmartDashboard.getNumber("Shoot Speed")` in `initialize()` | `UpToSpeedHopperShoot.java` | stage-2 |

> **Implementation note — H-3/M-4:** `VisionEstimate` only exposes `getPose()` and `getTimestamp()` — it has no `targetsUsed()` accessor. Vision filtering uses distance between the vision pose estimate and current odometry estimate.

---

## Stage 3: Pre-Integration Cleanup ✅

**Risk: LOW — Purely subtractive — No control logic changes**

Closes the remaining dead-code gaps from Stages 1–2 and removes all climber code before the Hackbots integration begins.

> **Original plan note:** The Stage 3 slot originally described "Shoot-on-the-Move Accuracy Fixes" — measuring real TOF values, wiring `RangeFinder.getRotAdder()`, and fixing radial speed correction. These items were superseded: `RangeFinder` is deleted in Stage 5 and replaced by `ToFAim`'s built-in convergence loop. TOF measurement on the physical robot now belongs in Stage 8-9/8-10.

### Items Completed

| ID | Description | File | Commit |
|----|-------------|------|--------|
| — | Remove dead `RangeFinder rangeFinder` field + import from `RobotContainer.java` | `RobotContainer.java` | stage-3 |
| — | Remove dead `RangeFinder` import from `UpToSpeedHopperShoot.java` | `UpToSpeedHopperShoot.java` | stage-3 |
| — | Delete `Climber.java`, `ClimbPole.java` | Multiple | stage-3 |
| — | Remove all climber bindings, fields, and `NamedCommands` from `RobotContainer.java` | `RobotContainer.java` | stage-3 |
| — | Delete `ClimberConstants` from `Constants.java` | `Constants.java` | stage-3 |

> **DIO pin 4 conflict resolved:** The README listed DIO 4 for both the turret limit switch and the climber limit switch. Removing the climber in Stage 3 eliminates this conflict. DIO 4 belongs exclusively to the turret now.

---

## Stage 4: Hackbots Integration — Files & Compile ✅

**Risk: MEDIUM — Copy and simplify only — No existing code modified**

**Source:** `C:\projects\Gearcats\Hackbots\2026_Rebuilt\CompBot\`

### Items Completed

| Description | File(s) | Commit |
|-------------|---------|--------|
| Add `frc.robot.util` package: `OnboardLogger`, `StatusSignalUtil` | `util/` | stage-4 |
| Add `frc.robot.aiming` package (7 files): `AimParams`, `AimStrategy`, `AimConstraints`, `AimMeasurement`, `ToFAim`, `PhysicsAim`, `TuneAim` | `aiming/` | stage-4 |
| Add `frc.robot.subsystems.shooter` package (5 files): `Shooter`, `ShooterIO`, `ShooterIOHardware`, `ShooterIOSim`, `ShooterConstants` (CAN IDs = 0) | `subsystems/shooter/` | stage-4 |
| Add Hackbots turret IO layer: `TurretIO`, `TurretConstants` (single encoder, CAN IDs = 0), `TurretIOHardware` (no CRT), `TurretIOSim`, `TurretIODisabled` | `subsystems/turret/` | stage-4 |
| Add Stage 4 stub commands: `AimPrep`, `ShootWhenReady` | `commands/` | stage-4 |
| Add `ROBOT_TO_TURRET_BASE` placeholder to `Constants.TurretConstants` | `Constants.java` | stage-4 |
| Zero compile errors verified | — | stage-4 |

> **Implementation notes:**
> - Hackbots `Turret.java` imports `StateManager` and `Superstructure` — deferred to Stage 5 to avoid breaking existing compilation. Old Gearcats `Turret.java` kept alongside new IO files for Stage 4.
> - `TurretIOHardware.java` simplified: single absolute CANcoder (no CRT math, no YAMS dependency). Calibrate reads `encoder.getAbsolutePosition()` and calls `motor.setPosition()`.
> - `AimPrep` and `ShootWhenReady` written as stubs; wired to `RobotStateMachine` in Stage 5.

---

## Stage 5: Hackbots Integration — State Machine Wire-Up ✅

**Risk: HIGH — Replaces core shooting subsystems — Largest single change**

> **Architecture decision:** Keep the Gearcats `RobotStateMachine` singleton. Adapt Hackbots `StateManager` logic into `RobotStateMachine` methods — less disruptive than adopting the full Superstructure pattern.

### Items Completed

| Description | File(s) | Commit |
|-------------|---------|--------|
| Replace `Flywheel m_Flywheel` → `Shooter m_Shooter` (mode-aware: `ShooterIOHardware` / `ShooterIOSim`) | `RobotStateMachine.java` | stage-5 |
| Add `getAimParams()` via `ToFAim` (Tag3d target, turret Pose3d, field velocity `Translation2d`) | `RobotStateMachine.java` | stage-5 |
| Add `isShootReady()` delegating to `Shooter.tracked()` | `RobotStateMachine.java` | stage-5 |
| Remove `updateTargetPose()` and `RangeFinder` dependency | `RobotStateMachine.java` | stage-5 |
| Replace Gearcats `Turret.java` with Hackbots version (uses `TurretIO` + `StateManager`) | `subsystems/turret/Turret.java` | stage-5 |
| Create `StateManager` bridge — adapts `robotPose()`, `aimParams()`, `shootReady` Trigger to `RobotStateMachine` | `superstructure/StateManager.java` | stage-5 |
| Create `LocalizationConstants` stub — turret camera offset placeholder for Stage 8-2 | `vision/localization/LocalizationConstants.java` | stage-5 |
| Update `AimPrep` — parallel `turret.track(state)` + `shooter.shoot(state::aimParams)` | `commands/AimPrep.java` | stage-5 |
| Update `ShootWhenReady` — `waitUntil(rsm::isShootReady)` then feed hopper | `commands/ShootWhenReady.java` | stage-5 |
| Simplify `SysIDUtil` — remove Flywheel/old Turret constructors (re-implement Stage 8) | `utility/SysIDUtil.java` | stage-5 |
| Delete 12 superseded files (see list below) | Multiple | stage-5 |
| Update `RobotContainer` — Shooter wiring, new `Turret(TurretIO)`, stub NamedCommands for Stage 6 | `RobotContainer.java` | stage-5 |
| Zero compile errors verified | — | stage-5 |

### Files Deleted in Stage 5

| File | Replaced By |
|------|-------------|
| `subsystems/turret/Flywheel.java` | `subsystems/shooter/Shooter.java` |
| `utility/RangeFinder.java` | `aiming/` package |
| `commands/CoolSnurbo.java` | Speed reduction internal to `Shooter` |
| `commands/UncoolSnurbo.java` | Dead stub — no replacement needed |
| `commands/UpToSpeedHopperShoot.java` | `commands/ShootWhenReady.java` |
| `commands/ShootingSequence.java` | `commands/AimPrep.java` |
| `commands/ShootingSequenceUTS.java` | `commands/AimPrep.java` |
| `commands/ShootFuel.java` | `AimPrep` + `ShootWhenReady` |
| `commands/AlignTurretToHub.java` | Turret tracking inside `AimPrep` |
| `commands/SetTurretAngle.java` | `Turret.home()` / `Turret.forwards()` |
| `commands/BurstFire.java` | Superseded composite — no replacement |
| `commands/MoveTurret.java` | Deleted — called old `Turret.setSpeed()` API; Stage 6 adds new manual jog |

> **API correction:** `ChassisSpeeds` has **no** `toTranslation2d()` method. `getAimParams()` uses `new Translation2d(fs.vxMetersPerSecond, fs.vyMetersPerSecond)` instead. The original plan stub was incorrect.

> **StateManager pattern:** Rather than creating a full `Superstructure` class, `StateManager` is a thin bridge class in `frc.robot.superstructure` that delegates to `RobotStateMachine.getInstance()`. This lets the Hackbots `Turret.java` compile unchanged while keeping the Gearcats singleton architecture.

---

## Stage 6: Hackbots Integration — RobotContainer & Autonomous

**Risk: HIGH — Changes controller bindings and auto paths**

### 6-1: Update controller bindings

| Button / Axis | Current State | Target State |
|---------------|---------------|--------------|
| Gunner Left Trigger | `AimPrep` stub (parallel turret track + shooter) | Wire fully — confirm turret tracks, hood adjusts |
| Gunner Left Bumper | `ShootWhenReady` stub | Wire fully — confirm feeds only when ready |
| POV Right / Left | Removed (MoveTurret deleted in Stage 5) | Add new manual jog: `m_turret.run(() -> io.setSpeed(±0.15))` or equivalent |
| POV Up / Down | Unused | Wire `AimPrep` tracking-only (no feed) |
| Driver Right Bumper | `m_shooter.reverse()` | Confirm OK or adjust |
| Drive speed axes | No speed modifier ✅ | Confirm drive feels correct without modifier |

### 6-2: Update NamedCommands — preserve string names so .auto files don't need editing

All shooting NamedCommands are currently `Commands.none()` stubs. Replace with real commands:

| NamedCommand String | New Registration |
|---------------------|-----------------|
| `"ShootFuel"`, `"ShootFuel3s"`, `"ShootFuel5s"`, `"ShootFuel7s"`, `"ShootFuel10s"` | `AimPrep` + `ShootWhenReady` |
| `"NewShootFuel3s"`, `"NewShootFuel4s"`, `"NewShootFuel5s"`, `"NewShootFuel8s"`, `"NewShootFuel10s"` | `AimPrep` + `ShootWhenReady` |
| `"ManualShootFuel3s"` | `ShootWhenReady` only (no turret tracking) |
| `"AlignTurret"`, `"AlignTurret1s"` | `AimPrep` tracking-only (no feed) |
| `"TrenchStartAngle"` | `m_turret.home()` |
| `"SpeedUp"` | `m_shooter.shoot(() -> AimParams.impossible())` warm-up or remove |

> Climber NamedCommands (`"Climb"`, `"ClimbUp2s"`, `"ClimbDown2s"`) were removed in Stage 3. Do not re-register.

### 6-3: Apply Stage 2 logging conventions to new subsystems

Add SmartDashboard entries for visibility in AdvantageScope:

```java
// Shooter
SmartDashboard.putNumber("Shooter/HoodAngleDeg",  /* inputs.hoodAngle */);
SmartDashboard.putNumber("Shooter/VelocityRPS",   /* inputs.shooter1Velocity */);
SmartDashboard.putBoolean("Shooter/IsTracked",    m_shooter.tracked(rsm::getAimParams).getAsBoolean());

// Aiming
SmartDashboard.putString("Aiming/Status",         rsm.getAimParams().status.toString());
SmartDashboard.putNumber("Aiming/TargetYawDeg",   rsm.getAimParams().yaw.getDegrees());
SmartDashboard.putNumber("Aiming/TargetPitchDeg", rsm.getAimParams().pitch.getDegrees());
```

### Stage 6 Verification ✓

1. Robot enables in Teleop — no crashes
2. Gunner left trigger: turret tracks hub, hood adjusts with distance
3. `ShootWhenReady` releases hopper only when `Robot/IsShootReady` is true
4. Manual turret jog (POV left/right) works
5. At least one PathPlanner auto path runs without exception
6. Drive speed NOT reduced during shooting
7. `Shooter/IsTracked`, `Aiming/Status` visible in AdvantageScope

---

## Stage 7: Lead Compensation Enhancement

**Risk: MEDIUM — New capability — Depends on Stage 6 verified + Stage 8-9 TOF data**

> **Code comparison finding:** archive2025 scored 82% SOTM capability vs 2026_Rebuilt at 28%. This stage closes that gap using archive2025's iterative lead logic combined with Hackbots' physics-grounded TOF.

### 7-1: Create `LeadCompensator.java` in `frc.robot.aiming`

```java
public class LeadCompensator {
    public static Pose3d computeLeadTarget(
            Pose3d hubPose, Pose3d shooterPose,
            Translation2d fieldVelocity,
            AimStrategy aimStrategy, AimConstraints constraints) {

        Pose3d best = hubPose;
        for (int i = 0; i < 5; i++) {
            AimParams params = aimStrategy.update(best, shooterPose, fieldVelocity);
            double tof = params.tof;
            best = new Pose3d(
                hubPose.getX() - (fieldVelocity.getX() * tof),
                hubPose.getY() - (fieldVelocity.getY() * tof),
                hubPose.getZ(),
                hubPose.getRotation());
        }
        return best;
    }
}
```

5 iterations converges in 3–4 at any realistic FRC speed. TOF from the physics solver, not a lookup table.

### 7-2: Wire into `RobotStateMachine.getAimParams()`

```java
public AimParams getAimParams() {
    ChassisSpeeds fs = getFieldSpeeds();
    Translation2d fieldVel = (fs != null)
        ? new Translation2d(fs.vxMetersPerSecond, fs.vyMetersPerSecond)
        : Translation2d.kZero;
    // NOTE: ChassisSpeeds has no toTranslation2d() — use Translation2d constructor directly
    Pose3d leadTarget = LeadCompensator.computeLeadTarget(
        Tag_POSE2D, new Pose3d(turretPose), fieldVel, m_tofAim, kScoringConstraints);
    return m_tofAim.update(leadTarget, new Pose3d(turretPose), fieldVel);
}
```

### 7-3: Add telemetry

```java
SmartDashboard.putNumber("Aiming/LeadOffsetXM", leadTarget.getX() - Tag_POSE2D.getX());
SmartDashboard.putNumber("Aiming/LeadOffsetYM", leadTarget.getY() - Tag_POSE2D.getY());
```

> **Rotation correction constants** (`0.000725312`, `2.82988`) are from Hackbots' calibration data. Re-validate against Gearcats' shooter geometry before applying.

### Stage 7 Verification ✓

1. `Aiming/LeadOffsetXM/YM` non-zero when moving, zero when stationary
2. Lateral shot at 2 m/s: hits hub
3. Radial toward hub at 2 m/s: hits hub
4. Radial away from hub at 2 m/s: hits hub
5. Stationary shots unchanged

---

## Stage 8: Hardware Wiring, Tuning & Measurement

**Risk: HARDWARE — Blocked by physical build**

| # | Item | How to Complete |
|---|------|-----------------|
| 8-1 | **Assign CAN IDs** — all new turret/shooter motors and encoders | Update all placeholder 0 values. Prior IDs: Turret=12, Flywheel R=13 L=14, Hopper=22/23, Intake=20/21, Climber=25 (now free). CANdle=50 |
| 8-2 | **Turret mounting offset** — `ROBOT_TO_TURRET_BASE` + `LocalizationConstants.kTurretAoRToTurretCameraOffset` | Measure from CAD or assembled robot |
| 8-3 | **Absolute encoder zero alignment** — `TurretConstants.kEncoderConfig.MagnetOffset` | Place magnet so encoder reads 0.0 when turret faces forward. Verify with Tuner X |
| 8-4 | **Hood CANcoder zero alignment** — `HoodConstants.kCANcoderConfig.MagnetOffset` | Set offset so hood reads 0 at minimum angle |
| 8-5 | **Confirm turret travel ≤ ±180°** | Drive to each soft-limit stop physically. If >360° total, single absolute encoder is insufficient |
| 8-6 | **Set hood angle range** — `AimConstraints` min/max pitch | Drive hood to physical limits; record encoder values at each end |
| 8-7 | **Re-characterize PID and MotionMagic gains** | Use SysID or Tuner X live tuning. Hackbots values are starting points only. Re-implement `SysIDUtil(Shooter)` and `SysIDUtil(Turret)` with new APIs |
| 8-8 | **Measure real TOF values** | 240 fps+ video at 6–12 distances from 1.5–6.5 m. Use `TuneAim` strategy. Record 8–12 points, then switch to `ToFAim` |
| 8-9 | **Validate lead compensation** | Drive at known speed in all directions. Confirm hits with logged `Aiming/LeadOffset` values |
| 8-10 | **Add intake deploy soft limit** (Issue L-5) | Add TalonFX soft limit or timer cutoff |

> **DIO Pin 4** — Turret home limit switch. Conflict with climber (Issue 8-8 in original plan) is **resolved** — climber removed in Stage 3.

---

## Appendix A — ISSUES.md Resolution Map

| ID | Severity | Issue | Stage | Resolution |
|----|----------|-------|-------|------------|
| C-1 | Critical | `System.gc()` every 100ms | 0 | Deleted |
| C-2 | Critical | 10+ SmartDashboard writes every loop in StateMachine | 0 | Rate-limited 10 Hz |
| C-3 | Critical | New `Pose2d` objects every loop | 0 | Rate-limited 10 Hz |
| C-4 | Critical | 6 SmartDashboard writes in `AlignTurretToHub.execute()` | 2 | Rate-limited 10 Hz — file deleted Stage 5 |
| H-1 | High | 4 CAN reads in `getChassisSpeeds()` | 0 | `getState().Speeds` — CTRE cached |
| H-2 | High | `getVisionEst()` called twice per camera | 2 | Cached in local variable |
| H-3 | High | No vision quality filtering | 2 | Distance-from-odometry filtering |
| H-4 | High | SmartDashboard writes every loop in Turret/Flywheel | 2 | Rate-limited 10 Hz with rename |
| H-5 | High | `CoolSnurbo` missing `addRequirements()` | 1 | Fixed — file superseded Stage 5 |
| H-6 | High | Climber motor permanently commented out | 3 | All climber code removed |
| H-7 | High | Non-thread-safe singleton | 0 | Eager initialization |
| M-1 | Medium | Turret PID Slot 1 gains default to zero | 8-7 | Add constant fallback |
| M-2 | Medium | `setPosition()` no bounds check | 0 | `MathUtil.clamp()` added — Hackbots Turret uses soft limits |
| M-3 | Medium | `RobotConfig` null if load fails | 0 | Null safety + error message |
| M-4 | Medium | Static vision std devs trust all equally | 2 | Dynamic distance-scaled std devs |
| M-5 | Medium | SmartDashboard read every `execute()` | 2 | Read once in `initialize()` — file superseded Stage 5 |
| L-1 | Low | Dead `m_motor.getConfigurator()` | 1 | Removed |
| L-2 | Low | Double semicolon in `RunIntake.java` | 1 | Removed |
| L-3 | Low | Duplicate imports in `Vision.java` | 1 | Removed |
| L-4 | Low | `kS` TODO — static friction | 8-7 | Verify at tuning |
| L-5 | Low | Intake deploy no soft limit | 8-10 | Add TalonFX soft limit or timer cutoff |

---

## Appendix B — File Change Summary

| File | Stages | Change Type |
|------|--------|-------------|
| `Robot.java` | 0, 2 | Delete System.gc(); add logConsoleOutput(); add system health |
| `RobotStateMachine.java` | 0, 2, 5 | Rate-limit; cache speeds; eager singleton; rename keys; replace Flywheel with Shooter; add getAimParams/isShootReady |
| `Constants.java` | 0, 3, 4 | RobotConfig null safety; delete ClimberConstants; add ROBOT_TO_TURRET_BASE placeholder |
| `Turret.java` (old Gearcats) | 0, 1, 2 | Bounds clamp; remove dead call; rename + rate-limit; hardware telemetry — **deleted Stage 5** |
| `Flywheel.java` | 1, 2 | Remove dead imports; rename + rate-limit; hardware telemetry — **deleted Stage 5** |
| `AlignTurretToHub.java` | 0, 2 | Rate-limit execute(); rename keys — **deleted Stage 5** |
| `Intake.java` | 2 | Rename key; add hardware telemetry |
| `Hopper.java` | 2 | Add periodic() |
| `Climber.java` | 3 | **Deleted** |
| `ClimbPole.java` | 3 | **Deleted** |
| `Vision.java` | 1, 2 | Remove duplicate imports; cache result; add filtering |
| `UpToSpeedHopperShoot.java` | 2, 3 | Read speed once; remove dead import — **deleted Stage 5** |
| `RunIntake.java` | 1 | Fix double semicolon |
| `RobotContainer.java` | 1, 3, 5, 6 | Remove Limelight; remove dead code + climber; new Turret/Shooter wiring; Stage 6 bindings |
| `RangeFinder.java` | 5 | **Deleted** (Stage 3 removes dead instance; class deleted Stage 5) |
| `LimelightIO.java` | 1 | **Deleted** — replaced by PhotonVisionIO |
| `LimelightHelpers.java` | 1 | **Deleted** — vendor library no longer needed |
| `CoolSnurbo.java` | 1 | Fix addRequirements — **deleted Stage 5** |
| `UncoolSnurbo.java` | 5 | **Deleted** — empty dead stub |
| `BurstFire.java` | 5 | **Deleted** — composite referencing deleted commands |
| `MoveTurret.java` | 5 | **Deleted** — used old `Turret.setSpeed()` API; new jog command added Stage 6 |
| `SetTurretAngle.java` | 5 | **Deleted** — replaced by `Turret.home()` |
| `ShootFuel.java` | 5 | **Deleted** — replaced by AimPrep + ShootWhenReady |
| `ShootingSequence.java` | 5 | **Deleted** — replaced by AimPrep + ShootWhenReady |
| `ShootingSequenceUTS.java` | 5 | **Deleted** — replaced by AimPrep + ShootWhenReady |
| `SysIDUtil.java` | 5 | Simplified — old Flywheel/Turret constructors removed; re-implement Stage 8 |
| `aiming/*.java` (7 files) | 4 | **New** — copied from Hackbots |
| `aiming/LeadCompensator.java` | 7 | **New** — ported from archive2025 logic |
| `util/OnboardLogger.java` | 4 | **New** — copied from Hackbots |
| `util/StatusSignalUtil.java` | 4 | **New** — copied from Hackbots |
| `subsystems/shooter/*.java` (5 files) | 4 | **New** — copied from Hackbots |
| `subsystems/turret/TurretIO.java` | 4 | **New** — Hackbots interface |
| `subsystems/turret/TurretConstants.java` | 4 | **New** — Hackbots version; single encoder; CAN IDs = 0 |
| `subsystems/turret/TurretIOHardware.java` | 4 | **New** — Hackbots version simplified: no CRT math |
| `subsystems/turret/TurretIOSim.java` | 4 | **New** — Hackbots version |
| `subsystems/turret/TurretIODisabled.java` | 4 | **New** — Hackbots version |
| `subsystems/turret/Turret.java` | 5 | **Replaced** — Hackbots version (SubsystemBase + TurretIO) |
| `commands/AimPrep.java` | 4, 5 | **New** stub; wired Stage 5 |
| `commands/ShootWhenReady.java` | 4, 5 | **New** stub; wired Stage 5 |
| `superstructure/StateManager.java` | 5 | **New** — bridges Hackbots Turret API to RobotStateMachine |
| `vision/localization/LocalizationConstants.java` | 5 | **New** — stub; turret camera offset placeholder |

---

## Appendix C — Hardware Reference

| CAN ID | Device | Role | Status |
|--------|--------|------|--------|
| 1–4 | SPARK MAX | Swerve drive | Active |
| 5–8 | SPARK MAX | Swerve steer | Active |
| 12 | TalonFX | Turret yaw (old) | Replaced Stage 5 |
| 13 | TalonFX | Flywheel right (old) | Replaced Stage 5 |
| 14 | TalonFX | Flywheel left (old) | Replaced Stage 5 |
| 20 | TalonFX | Intake roller | Active |
| 21 | TalonFX | Intake deploy | Active |
| 22 | TalonFX | Hopper indexer | Active |
| 23 | TalonFX | Hopper kicker | Active |
| 25 | — | (was Climber — now free) | Removed Stage 3 |
| 50 | CANdle | LEDs — 177 strip + 8 onboard | Active |
| TBD | TalonFX | Turret yaw (new) | Stage 8-1 |
| TBD | CANcoder | Turret absolute encoder (single) | Stage 8-1 |
| TBD | TalonFX×2 | Shooter motors (new) | Stage 8-1 |
| TBD | TalonFX | Hood motor (new) | Stage 8-1 |
| TBD | CANcoder | Hood encoder (new) | Stage 8-1 |

**DIO Pin 4** — Turret home limit switch. Climber conflict resolved in Stage 3.

---

*Gearcats FRC Team 6500 — Rebuilt2026 Master Development Plan — Branch: leto — Updated 2026-09-05*

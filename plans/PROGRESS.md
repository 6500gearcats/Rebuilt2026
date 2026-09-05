# Rebuilt2026 Refactor — Implementation Progress

**Branch:** `leto`  
**Plan file:** `Rebuilt2026_MasterPlan.md`  
**Last updated:** 2026-09-05

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started
- ❌ Blocked

---

## Stage 0 — Critical Performance Fixes (Loop Overruns)

| ID | Description | File | Status | Commit |
|----|-------------|------|--------|--------|
| C-1 | Delete `System.gc()` + `m_gcTimer` | `Robot.java` | ✅ | stage-0 |
| C-2 | Rate-limit 10+ SmartDashboard writes in `periodic()` | `RobotStateMachine.java` | ✅ | stage-0 |
| C-3 | Rate-limit `Pose2d`/`Rotation2d` allocations | `RobotStateMachine.java` | ✅ | stage-0 |
| C-4 | Rate-limit 8 SmartDashboard writes in `AlignTurretToHub.execute()` | `AlignTurretToHub.java` | ✅ | stage-0 |
| H-1 | Cache `ChassisSpeeds` — replace 4 module-state reads with `getState().Speeds` | `RobotStateMachine.java` | ✅ | stage-0 |
| H-7 | Fix non-thread-safe singleton — eager init | `RobotStateMachine.java` | ✅ | stage-0 |
| M-2 | Add `MathUtil.clamp()` bounds check to `Turret.setPosition()` | `Turret.java` | ✅ | stage-0 |
| M-3 | Improve `RobotConfig` null-safety error message | `Constants.java` | ✅ | stage-0 |

---

## Stage 1 — Dead Code Removal & Limelight Deprecation

| ID | Description | File | Status | Commit |
|----|-------------|------|--------|--------|
| L-1 | Remove dead `getConfigurator()` call | `Turret.java` | ✅ | stage-1 |
| L-2 | Remove double semicolon | `RunIntake.java` | ✅ | stage-1 |
| L-3 | Remove duplicate imports | `Vision.java` | ✅ | stage-1 |
| H-5 | Add `addRequirements()` to `CoolSnurbo` | `CoolSnurbo.java` | ✅ | stage-1 |
| H-6 | Investigate commented-out climber motor | `Climber.java` | ✅ | stage-1 |
| — | Deprecate all Limelight code (`LimelightIO.java`, `LimelightHelpers.java`) | Multiple | ✅ | stage-1 |
| — | Remove dead Limelight references from `RobotContainer.java` | `RobotContainer.java` | ✅ | stage-1 |

---

## Stage 2 — Logging & Telemetry Improvements

| ID | Description | File | Status | Commit |
|----|-------------|------|--------|--------|
| — | Add `DataLogManager.logConsoleOutput(true)` | `Robot.java` | ✅ | stage-2 |
| — | Add system health (battery voltage, CAN utilization, RSL) | `Robot.java` | ✅ | stage-2 |
| — | Rename SmartDashboard keys to `"Subsystem/Key"` format | Multiple | ✅ | stage-2 |
| — | Add motor hardware telemetry (current, voltage, temp) to Flywheel/Turret | Multiple | ✅ | stage-2 |
| — | Add `Hopper.periodic()` with motor telemetry | `Hopper.java` | ✅ | stage-2 |
| — | Add Intake motor hardware telemetry | `Intake.java` | ✅ | stage-2 |
| H-2 | Fix `getVisionEst()` called twice per loop — cache in local variable | `Vision.java` | ✅ | stage-2 |
| H-3 | Add vision measurement filtering (reject >4 m from odometry) | `Vision.java` | ✅ | stage-2 |
| H-4 | Rate-limit Turret/Flywheel periodic SmartDashboard writes | Multiple | ✅ | stage-2 |
| M-4 | Scale vision std devs with distance via `addVisionMeasurement` overload | `Vision.java` | ✅ | stage-2 |
| M-5 | Cache `SmartDashboard.getNumber("Shoot Speed")` in `initialize()` | `UpToSpeedHopperShoot.java` | ✅ | stage-2 |

---

## Stage 3 — Pre-Integration Cleanup

| ID | Description | File | Status | Commit |
|----|-------------|------|--------|--------|
| — | Remove dead `RangeFinder rangeFinder` field + import from `RobotContainer.java` | `RobotContainer.java` | ✅ | stage-3 |
| — | Remove dead `RangeFinder` import from `UpToSpeedHopperShoot.java` | `UpToSpeedHopperShoot.java` | ✅ | stage-3 |
| — | Delete `Climber.java`, `ClimbPole.java` | Multiple | ✅ | stage-3 |
| — | Remove all climber bindings, fields, and `NamedCommands` from `RobotContainer.java` | `RobotContainer.java` | ✅ | stage-3 |
| — | Delete `ClimberConstants` from `Constants.java` | `Constants.java` | ✅ | stage-3 |

---

## Stage 4 — Hackbots Integration (Files & Compile)

| Description | File(s) | Status | Commit |
|-------------|---------|--------|--------|
| Add `frc.robot.util` package: `OnboardLogger`, `StatusSignalUtil` | `util/` | ✅ | stage-4 |
| Add `frc.robot.aiming` package (7 files): `AimParams`, `AimStrategy`, `AimConstraints`, `AimMeasurement`, `ToFAim`, `PhysicsAim`, `TuneAim` | `aiming/` | ✅ | stage-4 |
| Add `frc.robot.subsystems.shooter` package (5 files): `Shooter`, `ShooterIO`, `ShooterIOHardware`, `ShooterIOSim`, `ShooterConstants` (CAN IDs = 0) | `subsystems/shooter/` | ✅ | stage-4 |
| Add Hackbots turret IO layer: `TurretIO`, `TurretConstants` (single encoder, CAN IDs = 0), `TurretIOHardware` (simplified — no CRT), `TurretIOSim`, `TurretIODisabled` | `subsystems/turret/` | ✅ | stage-4 |
| Add Stage 4 stub commands: `AimPrep`, `ShootWhenReady` | `commands/` | ✅ | stage-4 |
| Add `ROBOT_TO_TURRET_BASE` placeholder to `Constants.TurretConstants` | `Constants.java` | ✅ | stage-4 |
| Zero compile errors verified (`.\gradlew.bat compileJava`) | — | ✅ | stage-4 |

**Source:** https://github.com/hackbots-3414/2026_Rebuilt  
**Local clone:** `C:\projects\Gearcats\Hackbots\2026_Rebuilt\`

---

## Stage 5 — Hackbots State Machine Wire-Up

| Description | Status |
|-------------|--------|
| Migrate Hackbots `StateManager` logic into `RobotStateMachine` | ⬜ |
| Wire `ToFAim` / `PhysicsAim` strategy into shooting sequence | ⬜ |
| Integrate `DynamicMotionMagicVoltage` for hood/turret | ⬜ |

---

## Stage 6 — RobotContainer & Auto

| Description | Status |
|-------------|--------|
| Update `RobotContainer` with new Hackbots-sourced subsystem bindings | ⬜ |
| Verify PathPlanner `NamedCommand` strings are preserved | ⬜ |
| Verify all autos run with new subsystem structure | ⬜ |

---

## Stage 7 — Lead Compensation Enhancement

| Description | Status |
|-------------|--------|
| Implement `LeadCompensator` utility (5-iteration iterative lead calc) | ⬜ |
| Replace lookup-table TOF approach from archive2025 | ⬜ |
| Validate against `InterpolatingDoubleTreeMap` shot data | ⬜ |

---

## Stage 8 — Hardware, Wiring & Tuning

| ID | Description | Status |
|----|-------------|--------|
| 8-1 | Tune Turret Slot0/Slot2 PID on assembled robot | ⬜ |
| 8-2 | Measure and set `kS` TODO in Flywheel/Turret | ⬜ |
| 8-3 | Add intake deploy soft limit | ⬜ |
| 8-4 | Verify PhotonVision camera names match physical layout | ⬜ |

---

## Notes

- **DIO pin 4**: Previously conflicted between turret limit switch and climber — resolved by climber removal in Stage 3. Turret now has exclusive use of DIO 4.
- **SPARK MAX swerve**: Drive/steer motors (IDs 1-8) are REV SPARK MAX — TalonFX telemetry APIs do NOT apply to swerve motors.
- **Constants.AutoConstants.config**: This field is unused in the codebase — PathPlanner config is loaded directly in `CommandSwerveDrivetrain.configureAutoBuilder()`.


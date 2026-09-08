# Rebuilt2026 — Documentation Progress

Tracks execution of the documentation improvement plan. See `doc_plan.md` for full rationale and task details.

**Branch:** `leto`  
**Last updated:** 2026-09-08 (D-3 complete)

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started
- ⏭ Skipped (auto-generated or already meets bar)

---

## D-1 — Priority 1: Complex Logic, Actively Misleading or Zero Docs

| Task | File | Status | Commit |
|------|------|--------|--------|
| D1-1 | `aiming/PhysicsAim.java` | ✅ | `docs-d1` |
| D1-2 | `RobotStateMachine.java` (getState, private methods, enums) | ✅ | `docs-d1` |
| D1-3 | `Robot.java` (class, constructor, teleopPeriodic, lifecycle) | ✅ | `docs-d1` |
| D1-4 | `commands/RunHopper.java` | ✅ | `docs-d1` |
| D1-5 | `subsystems/vision/Vision.java` | ✅ | `docs-d1` |

---

## D-2 — Priority 2: Public API Gaps

| Task | Files | Status | Commit |
|------|-------|--------|--------|
| D2-1 | `ShooterIO.java`, `ShooterIOHardware.java`, `ShooterIOSim.java` | ✅ | `docs-d2` |
| D2-2 | `util/OnboardLogger.java` | ✅ | `docs-d2` |
| D2-3 | `subsystems/intake/Intake.java` | ✅ | `docs-d2` |
| D2-4 | `aiming/AimMeasurement.java` | ✅ | `docs-d2` |
| D2-5 | `subsystems/turret/TurretIOSim.java` | ✅ | `docs-d2` |

---

## D-3 — Priority 3: Moderate Gaps

| Task | Files | Status | Commit |
|------|-------|--------|--------|
| D3-1 | `subsystems/turret/TurretIODisabled.java` | ✅ | `docs-d3-drivetrain` |
| D3-2 | `aiming/TuneAim.java` | ✅ | `docs-d3-aiming` |
| D3-3 | `subsystems/drivetrain/CommandSwerveDrivetrain.java` | ✅ | `docs-d3-drivetrain` |
| D3-4 | `subsystems/turret/TurretIOHardware.java` | ✅ | `docs-d3-drivetrain` |
| D3-5 | `commands/RunIntake.java` | ✅ | `docs-d3-commands` |
| D3-6 | `commands/ControllerRumble.java` | ✅ | `docs-d3-commands` |
| D3-7 | `commands/StaggerHopper.java` | ✅ | `docs-d3-commands` |
| D3-8 | `util/StatusSignalUtil.java` | ✅ | `docs-d3-util` |
| D3-9 | `subsystems/shooter/ShooterConstants.java` | ✅ | `docs-d3-constants` |
| D3-10 | `VisionIO.java`, `VisionEstimate.java`, `PhotonVisionIO.java`, `PhotonVisionSimIO.java` | ✅ | `docs-d3-vision` |
| D3-11 | `aiming/AimStrategy.java` | ✅ | `docs-d3-aiming` |
| D3-12 | `Constants.java` inner classes, `Main.java` | ✅ | `docs-d3-constants` |
| D3-13 | `subsystems/drivetrain/SysIDUtil.java`, `subsystems/shooter/ShooterValuesSenable.java` | ✅ | `docs-d3-util` |

---

## Summary

| Stage | Files | Status |
|-------|-------|--------|
| D-1 | 5 files | ✅ |
| D-2 | 5 files / 7 actual files | ✅ |
| D-3 | 13 tasks / ~20 actual files | ✅ |

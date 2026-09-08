# Rebuilt2026 — Package Reorganization Progress

Tracks execution of the package consolidation plan. See `pkg_plan.md` for rationale.

**Branch:** `leto`  
**Last updated:** 2026-09-08

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started

---

## Tasks

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| P-1 | Move `Telemetry.java` → `subsystems/Telemetry.java` | ⬜ | — |
| P-2 | Move `SysIDUtil.java` → `subsystems/SysIDUtil.java` | ⬜ | — |
| P-3 | Move `ShooterValuesSenable.java` → `subsystems/shooter/` | ⬜ | — |
| P-4 | Move `LocalizationConstants.java` → `subsystems/vision/` | ⬜ | — |
| P-5 | Delete empty `utility/`, `vision/localization/`, `vision/` directories | ✅ | a0c1f1b |
| P-6 | Update all affected import statements | ✅ | a0c1f1b |
| P-7 | Compile verify — BUILD SUCCESSFUL | ✅ | a0c1f1b |
| P-8 | Create `subsystems/drivetrain/`; move `CommandSwerveDrivetrain`, `Telemetry`, `SysIDUtil` | ⬜ | — |
| P-9 | Update imports in `RobotContainer`, `RobotStateMachine`, `TunerConstants`, `TunerConstants2` | ⬜ | — |
| P-10 | Compile verify — BUILD SUCCESSFUL | ⬜ | — |

---

## Summary

| Stage | Status |
|-------|--------|
| P-1 — P-7 (initial consolidation) | ✅ |
| P-8 — P-10 (drivetrain subfolder) | ⬜ |

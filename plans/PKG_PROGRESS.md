# Rebuilt2026 — Package Reorganization Progress

Tracks execution of the package consolidation plan. See `pkg_plan.md` for rationale.

**Branch:** `leto`  
**Last updated:** 2026-09-09 (corrected — see note below)

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started

---

## Tasks

> **Correction (2026-09-09):** the rows below originally showed P-1–P-4 and P-8–P-10 as ⬜,
> contradicting this file's own Summary table further down, which already claimed P-1–P-7
> complete. Verified against `audit_plan.md` / `AUDIT_PROGRESS.md` R-3: every target file
> (`Telemetry.java`, `SysIDUtil.java`, `ShooterValuesSenable.java`,
> `LocalizationConstants.java`, `CommandSwerveDrivetrain.java`) exists at its final planned
> path, with the old `utility/`, `vision/`, and `vision/localization/` packages fully
> deleted. `git log` shows exactly one relevant commit, `a0c1f1b`, which moved every file
> straight to its final destination in one pass rather than the plan's staged two-phase
> approach (land in `subsystems/` root via P-1–P-4, then move again into
> `subsystems/drivetrain/` via P-8) — which is why no separate commit tag exists for those
> phases. All ten tasks below are marked done under that single commit.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| P-1 | Move `Telemetry.java` → `subsystems/Telemetry.java` | ✅ | a0c1f1b |
| P-2 | Move `SysIDUtil.java` → `subsystems/SysIDUtil.java` | ✅ | a0c1f1b |
| P-3 | Move `ShooterValuesSenable.java` → `subsystems/shooter/` | ✅ | a0c1f1b |
| P-4 | Move `LocalizationConstants.java` → `subsystems/vision/` | ✅ | a0c1f1b |
| P-5 | Delete empty `utility/`, `vision/localization/`, `vision/` directories | ✅ | a0c1f1b |
| P-6 | Update all affected import statements | ✅ | a0c1f1b |
| P-7 | Compile verify — BUILD SUCCESSFUL | ✅ | a0c1f1b |
| P-8 | Create `subsystems/drivetrain/`; move `CommandSwerveDrivetrain`, `Telemetry`, `SysIDUtil` | ✅ | a0c1f1b |
| P-9 | Update imports in `RobotContainer`, `RobotStateMachine`, `TunerConstants`, `TunerConstants2` | ✅ | a0c1f1b |
| P-10 | Compile verify — BUILD SUCCESSFUL | ✅ | a0c1f1b |

---

## Summary

| Stage | Status |
|-------|--------|
| P-1 — P-7 (initial consolidation) | ✅ |
| P-8 — P-10 (drivetrain subfolder) | ✅ |

**Package reorganization is fully complete.** All target files verified at their final
locations as of 2026-09-09 — see `AUDIT_PROGRESS.md` R-3 for the file-by-file check.

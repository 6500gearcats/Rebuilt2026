# Rebuilt2026 — Root README Rewrite Progress

Tracks execution of `readme_plan.md`.

**Branch:** `leto`
**Last updated:** 2026-09-09 (complete)

---

| Task | Status | Commit |
|------|--------|--------|
| RM-1 | Research: gather ground truth | ✅ |
| RM-2 | Write new root `README.md` | ✅ |
| RM-3 | Shrink `plans/README.md` to a folder index | ✅ |
| RM-4 | Final sanity re-check of specific claims | ✅ |

## Findings during research (RM-1)

- **Real bug found, not just a doc issue:** `ShooterConstants.kMotor1Id` (set to 30 earlier
  this session in `logging_plan.md`) collided with `TunerConstants2`'s Pigeon2 IMU, also on
  CAN ID 30. Fixed separately in commit `52e0417` (moved to ID 36) before continuing the
  README — the original 30–35 range only checked against swerve drive/steer IDs (0–7), not
  the Pigeon.
- `LedCANdle.java` doesn't exist anywhere in `src/` — fully removed, not just the
  `RobotContainer` instance `cleanup.md` C-4 removed.
- REVLib is vendored and imported in `Constants.DriveConstants` (`SparkBaseConfig.IdleMode`)
  but not used to drive any actual hardware — the swerve is entirely CTRE TalonFX-based.
  Documented as legacy/unused rather than removed (removal wasn't in scope here).
- No DIO pins are used anywhere in the current codebase — the turret's single-CANcoder
  design has no limit-switch homing, unlike what the old README described.

## Findings during final check (RM-4)

- The old README claimed NetworkTables ran on port 1735. That's the legacy NT3 port; this
  codebase uses NT4 throughout (`NetworkTableInstance.getDefault()`), whose WPILib default
  is port 5810. Not set explicitly anywhere in code — corrected before publishing.

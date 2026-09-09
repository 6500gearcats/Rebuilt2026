# Rebuilt2026 — Motor Electrical Logging Progress

Tracks execution of the logging plan. See `logging_plan.md` for full rationale and task details.

**Branch:** `leto`
**Last updated:** 2026-09-09 (starting)

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started

---

| Task | Files | Status | Commit |
|------|-------|--------|--------|
| L-0 | `Robot.java` — wire `OnboardLogger.logAll()` into `robotPeriodic()` | ⬜ | |
| L-1 | `util/OnboardLogger.java` — add `registerEnergy()` | ⬜ | |
| L-2 | `subsystems/shooter/Shooter.java` — log voltage/current/temp + energy, 3 motors | ⬜ | |
| L-3 | `subsystems/turret/Turret.java` — log voltage/current/temp + energy, 1 motor | ⬜ | |
| L-4 | `subsystems/intake/Intake.java` — add voltage/supply current reads, migrate to OnboardLogger, energy, 2 motors | ⬜ | |
| L-5 | `subsystems/hopper/Hopper.java` — same as L-4, 2 motors | ⬜ | |
| L-6 | `subsystems/drivetrain/CommandSwerveDrivetrain.java` — expose + log all 8 module motors + energy | ⬜ | |

---

## Summary

| Stage | Motors covered | Status |
|-------|-----------------|--------|
| L-0/L-1 | (infrastructure fix, no motors) | ⬜ |
| L-2 | 3 (shooter1, shooter2, hood) | ⬜ |
| L-3 | 1 (turret) | ⬜ |
| L-4 | 2 (intake roller, deploy) | ⬜ |
| L-5 | 2 (hopper indexer, kicker) | ⬜ |
| L-6 | 8 (4 drive + 4 steer) | ⬜ |
| **Total** | **16 motors** | ⬜ |

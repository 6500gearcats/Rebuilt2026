# Rebuilt2026 — Motor Electrical Logging Progress

Tracks execution of the logging plan. See `logging_plan.md` for full rationale and task details.

**Branch:** `leto`
**Last updated:** 2026-09-09 (complete)

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started

---

| Task | Files | Status | Commit |
|------|-------|--------|--------|
| L-0 | `Robot.java` — wire `OnboardLogger.logAll()` into `robotPeriodic()` | ✅ | `03368ea` |
| L-1 | `util/OnboardLogger.java` — add `registerEnergy()` | ✅ | `03368ea` |
| L-2 | `subsystems/shooter/ShooterIO.java` — energy for 3 motors (voltage/current/temp already logged) | ✅ | `68dd95a` |
| L-3 | `subsystems/turret/TurretIO.java` — energy for 1 motor (voltage/current/temp already logged) | ✅ | `68dd95a` |
| L-4 | `subsystems/intake/Intake.java` — add voltage/supply current reads, migrate to OnboardLogger, energy, 2 motors | ✅ | `02d082e` |
| L-5 | `subsystems/hopper/Hopper.java` — same as L-4, 2 motors | ✅ | `02d082e` |
| L-6 | `subsystems/drivetrain/CommandSwerveDrivetrain.java` — expose + log all 8 module motors + energy | ✅ | `ce1a8eb` |
| L-6b (found during L-6) | `Robot.java` — `StatusSignalUtil.refreshAll()` was also never called; fixed | ✅ | `ce1a8eb` |

---

## Summary

| Stage | Motors covered | Status |
|-------|-----------------|--------|
| L-0/L-1 | (infrastructure fix, no motors) | ✅ |
| L-2 | 3 (shooter1, shooter2, hood) | ✅ |
| L-3 | 1 (turret) | ✅ |
| L-4 | 2 (intake roller, deploy) | ✅ |
| L-5 | 2 (hopper indexer, kicker) | ✅ |
| L-6 | 8 (4 drive + 4 steer) | ✅ |
| L-6b | (infrastructure fix, no motors) | ✅ |
| **Total** | **16 motors** | ✅ |

## Bugs found and fixed along the way (not originally scoped, but blocking)

1. **`OnboardLogger.logAll()` never called** (L-0) — every existing `OnboardLogger`
   registration in the codebase (Shooter's Hood Reference, Turret's Ready/Tracking, etc.)
   had been silently writing nothing since introduced.
2. **Original audit error** (caught before shipping) — `ShooterIOInputs`/`TurretIOInputs`
   already self-registered voltage/current/temperature; only energy was actually missing
   for those two subsystems. Plan corrected in place rather than over-building L-2/L-3.
3. **`StatusSignalUtil.refreshAll()` never called** (found during L-6) — the same bug
   pattern as #1, one layer down: bulk-registered CTRE signals (including Shooter/Turret's
   existing control-loop reads) only updated at CTRE's slow default background rate, not
   the 50 Hz loop rate. Fixed alongside L-6.

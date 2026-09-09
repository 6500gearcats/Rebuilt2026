# plans/ — Index

This folder holds planning and progress-tracking documents for Rebuilt2026 work. It does
not describe the robot itself — for architecture, subsystems, controls, and hardware, see
the **[repository root `README.md`](../README.md)**.

Writing or updating a plan here? Read **[`PLANNING_GUIDE.md`](PLANNING_GUIDE.md)** first —
process guidance on keeping a plan and its progress tracker in sync with the code, based on
real drift found in this repo's own plans.

---

## Active / reference

| File | Covers |
|---|---|
| [`Rebuilt2026_RefactorPlan.md`](Rebuilt2026_RefactorPlan.md) + [`PROGRESS.md`](PROGRESS.md) | The master plan — Stages 0–8, from loop-overrun fixes through the Hackbots integration to hardware bring-up. Start here for overall project history. |
| [`ISSUES.md`](ISSUES.md) | The original issue catalog (severity-ranked) that Stages 0–2 of the master plan resolve. |

## Completed work (each still holds a plan + progress pair for reference)

| File | Covers |
|---|---|
| [`cleanup.md`](cleanup.md) + [`CLEANUP_PROGRESS.md`](CLEANUP_PROGRESS.md) | Post-refactor dead-code removal (`RobotStateMachine`/`RobotContainer` cruft) before Stage 8. |
| [`pkg_plan.md`](pkg_plan.md) + [`PKG_PROGRESS.md`](PKG_PROGRESS.md) | Package reorganization — consolidated `util`/`utility`, `vision`/`subsystems/vision`, moved drivetrain files into `subsystems/drivetrain/`. |
| [`sim_plan.md`](sim_plan.md) + [`SIM_PROGRESS.md`](SIM_PROGRESS.md) | Simulation quality — Limelight removal, PhotonVision API updates, sim-state seeding for Shooter/Hopper/Intake. |
| [`doc_plan.md`](doc_plan.md) + [`DOC_PROGRESS.md`](DOC_PROGRESS.md) | Tutorial-level Javadoc pass across the codebase. |
| [`logging_plan.md`](logging_plan.md) + [`LOGGING_PROGRESS.md`](LOGGING_PROGRESS.md) | Per-motor voltage/current/temperature/energy logging (all 16 TalonFX) via `OnboardLogger`. |
| [`audit_plan.md`](audit_plan.md) + [`AUDIT_PROGRESS.md`](AUDIT_PROGRESS.md) | Verified every plan above against current code — found and documented three tracker/code sync issues. |
| [`fixit_plan.md`](fixit_plan.md) + [`FIXIT_PROGRESS.md`](FIXIT_PROGRESS.md) | Corrected the three tracker inaccuracies the audit found. |
| [`readme_plan.md`](readme_plan.md) + [`README_PROGRESS.md`](README_PROGRESS.md) | Extracted and rewrote the stale architecture reference (formerly this file) into the repository root `README.md`. |
| [`review_plan.md`](review_plan.md) + [`REVIEW_PROGRESS.md`](REVIEW_PROGRESS.md) | Remediation for a full-codebase review (2026-09-09): 3 correctness bugs (mismatched AprilTag layouts, wrong pose-estimator kinematics, `Telemetry` never instantiated), aim-pipeline caching, a dead-code sweep, 7 logging additions, and 16 unit tests. Stages R1/R2/R4/R5/R6 done; R3 deferred by decision. See "Open work" below for what's left. |

## Open work

| Item | Where | What's needed |
|---|---|---|
| D-6 — `FieldZone`/`checkZone()` Y-boundary swap | `plans/REVIEW_PROGRESS.md` decision log | A human with the field diagram/CAD — the enum Javadoc and the code's actual return values are exactly swapped for `NEUTRAL_TOP`/`NEUTRAL_BOTTOM`; not resolvable from code alone |
| R1-A3 loop-timing verification | `plans/REVIEW_PROGRESS.md`, Stage R1 note | One sim run watching for overrun/NT4-flood warnings, now that `Telemetry.registerTelemetry` is wired to CTRE's 100 Hz odometry thread |
| Stage R3 (side-effect getters) | `plans/review_plan.md` | Deferred by decision — hygiene, not an active failure; revisit after competition or sooner if the schedule allows |

---

## Naming convention

`<topic>_plan.md` (lowercase) pairs with `<TOPIC>_PROGRESS.md` (uppercase), except the
master plan (`Rebuilt2026_RefactorPlan.md` / `PROGRESS.md`) and `ISSUES.md`, which predate
this convention. New plan/progress pairs should follow the lowercase/uppercase split.

# Rebuilt2026 — Plan Tracker Corrections

Fixes the three documentation inaccuracies found by `audit_plan.md` / `AUDIT_PROGRESS.md`.
All three are tracker-only corrections — no source code changes. Every inaccuracy found by
the audit traced to a plan or progress file not being updated after a later event (a file
deletion, a combined commit, or a deliberate reversal), never to the underlying code being
wrong, so this plan only touches the three affected markdown files.

Track execution in `FIXIT_PROGRESS.md`.

**Branch:** `leto`

---

## F-1 — `Rebuilt2026_RefactorPlan.md`: supersession notes + stale Stage 6 status

**Source finding:** R-1 in `AUDIT_PROGRESS.md`.

Two separate corrections in this one file:

1. **Top-level status table** (near the top of the file) marks Stage 6 "⬜ Next," but
   `PROGRESS.md` and current `RobotContainer.java` both confirm it shipped in commit
   `472c654`. Update the status cell to "✅ Done."
2. **Stage 2 per-stage table** claims telemetry was "added to Flywheel/Turret" and that
   `Hopper.periodic()`/Intake telemetry were complete. Add a note (matching the style
   Appendix B already uses elsewhere in this same file, e.g. "— file superseded Stage 5")
   to the rows whose target no longer exists or whose claimed completeness didn't survive:
   - Flywheel/Turret telemetry row — note `Flywheel.java` deleted Stage 5; current
     telemetry in `ShooterIO`/`TurretIO` arrived independently via the Stage 4 Hackbots
     copy, not from this edit.
   - Turret/Flywheel rate-limited `SmartDashboard` writes row (H-4) — note current
     `Turret.java` has no `SmartDashboard` calls in `periodic()` at all; superseded by the
     Stage 5 replacement.
   - Hopper/Intake telemetry rows — note these were stator-current-only until
     `logging_plan.md` (2026-09-09) added voltage, supply current, and energy.
   - M-5 row (`UpToSpeedHopperShoot` caching) — note the target file was deleted Stage 5,
     replaced by `ShootWhenReady.java`.

---

## F-2 — `PKG_PROGRESS.md`: mark the whole reorg done

**Source finding:** R-3 in `AUDIT_PROGRESS.md`.

The task table marks P-1 through P-4 and P-8 through P-10 as ⬜, but every target file was
verified at its final location with commit `a0c1f1b` as the sole relevant commit. Mark all
ten tasks ✅ under `a0c1f1b`, and update the Summary table to match. Add a one-line note
explaining the single-commit consolidation (the plan described a two-phase move; the actual
commit did it in one pass, which is why the original per-phase commit tags don't apply).

---

## F-3 — `SIM_PROGRESS.md`: record the deprecated-constructor decision

**Source finding:** R-4 in `AUDIT_PROGRESS.md`.

S2-2 currently reads as a completed migration ("Replace deprecated 3-arg constructor with
2-arg"). Per the user, this was superseded by a later, deliberate decision to keep the
original 3-arg `PhotonPoseEstimator(layout, strategy, transform)` constructor rather than
migrate it. Update the S2-2 row to record that decision and its rationale (rather than
leaving it worded as an open migration a future reader might attempt), and adjust S2-5's
"2 warnings remain" note so it doesn't imply that fixing S2-2 would reduce that count.

---

## Commit Strategy

One commit per file, since each is an independent, unrelated correction:

| Commit | File |
|---|---|
| `fixit-f1-master-plan` | `Rebuilt2026_RefactorPlan.md` |
| `fixit-f2-pkg-progress` | `PKG_PROGRESS.md` |
| `fixit-f3-sim-progress` | `SIM_PROGRESS.md` |

No compile step needed — markdown only.

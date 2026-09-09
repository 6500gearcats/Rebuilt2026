# Rebuilt2026 — Plan Completion Audit Progress

Tracks execution of the audit plan. See `audit_plan.md` for method and full rationale.
Read-only — no code changes in this pass.

**Branch:** `leto`
**Last updated:** 2026-09-09 (R-1 complete)

---

## Legend
- ✅ Verified — matches current code
- ⚠️ Stale — target file since replaced/deleted; substance may or may not survive elsewhere
- ❌ Contradicted — code does not match the claim
- ⬜ Not yet reviewed

---

| Stage | Plan reviewed | Status |
|-------|---------------|--------|
| R-1 | `Rebuilt2026_RefactorPlan.md` + `PROGRESS.md` + `ISSUES.md` | ✅ |
| R-2 | `cleanup.md` + `CLEANUP_PROGRESS.md` | ⬜ |
| R-3 | `pkg_plan.md` + `PKG_PROGRESS.md` | ⬜ |
| R-4 | `sim_plan.md` + `SIM_PROGRESS.md` | ⬜ |
| R-5 | `doc_plan.md` + `DOC_PROGRESS.md` | ⬜ |
| R-6 | `logging_plan.md` + `LOGGING_PROGRESS.md` | ⬜ |

---

## R-1 Findings — Master Plan + ISSUES.md

### Stage 0 (Critical Performance Fixes) — all ✅ Verified

| ID | Claim | Verified against |
|---|---|---|
| C-1 | Delete `System.gc()` + `m_gcTimer` | `Robot.java` — neither exists |
| C-2 | Rate-limit telemetry to 10 Hz | `RobotStateMachine.java:241` — `m_telemetryTimer.advanceIfElapsed(0.1)` gates pose/turretPose publishers |
| H-1 | Cache `ChassisSpeeds` | `RobotStateMachine.java:355-358` — `getChassisSpeeds()` returns `drivetrain.getState().Speeds` directly, no manual module reads |
| H-7 | Eager singleton | `RobotStateMachine.java:71` — `private static final RobotStateMachine instance = new RobotStateMachine();` |
| M-3 | `RobotConfig` error message | `Constants.java:191-199` — matches claim almost verbatim |

Stage 0 is real and holding up. C-3 and C-4 not independently re-checked this pass (low
risk — same file/pattern as C-2, and C-4's target `AlignTurretToHub.java` was deleted in
Stage 5 regardless).

### Stage 1 (Dead Code / Limelight) — Not independently re-checked

Plausible given Stage 4/5's wholesale replacement of the files these touched, but not
re-verified line by line this pass. Low risk — purely subtractive changes are easy to get
right and hard to silently regress.

### Stage 2 (Logging & Telemetry) — the flagged concern. Mixed: some items hold, several are
either stale or were never actually complete.

| Claim | Verdict | Detail |
|---|---|---|
| `DataLogManager.logConsoleOutput(true)` | ✅ | Present in `Robot.java` |
| System health at 10 Hz | ✅ | Present, `m_healthTimer.advanceIfElapsed(0.1)` |
| Rename keys to `Subsystem/Key` | ✅ (spot-checked) | Consistent in files read this session |
| H-2: `getVisionEst()` called once, not twice | ✅ | Current `Vision.java` calls it once into `est`, reuses for both the Kalman update and the gcc/gcd publish |
| H-3: reject vision >4 m from odometry | ✅ | `if (dist > 4.0) return;` present |
| M-4: distance-scaled vision std devs | ✅ | `0.1 + dist*0.05` formula present |
| "Add motor hardware telemetry (current, voltage, temp) to **Flywheel**/Turret" | ⚠️ **Stale** | `Flywheel.java` was deleted in Stage 5, replaced by `Shooter`/`ShooterIO`. The telemetry that exists today in `ShooterIOInputs` did not survive from this Stage 2 edit — it arrived independently via the Stage 4 Hackbots file copy. Stage 2's actual work on the old `Flywheel.java` no longer exists in any form. |
| H-4: "Rate-limit **Turret**/Flywheel periodic SmartDashboard writes at 10 Hz" | ❌ **Contradicted** (for current code) | Old `Turret.java` (the one this fix applied to) was replaced in Stage 5. Current `Turret.periodic()` contains zero `SmartDashboard` calls — it only calls `io.updateInputs(inputs)`. Telemetry now flows through `TurretIOInputs`'s self-registered `OnboardLogger`, which is **not rate-limited at all** — it logs every loop (confirmed this session), a materially different architecture than "rate-limited `SmartDashboard` writes." The claim describes a mechanism that no longer exists. |
| "Add `Hopper.periodic()` with motor telemetry" | ⚠️ **Stale/Incomplete** | `Hopper.periodic()` did exist pre-session with stator current only (no voltage, no supply current) — technically true but far short of "motor hardware telemetry." Fixed today in `logging_plan.md` L-5. |
| "Add Intake motor hardware telemetry" | ⚠️ **Stale/Incomplete** | Same pattern as Hopper — stator current only, pre-session. Fixed today in L-4. |
| M-5: cache `SmartDashboard.getNumber("Shoot Speed")` | ⚠️ **Stale** | Target file `UpToSpeedHopperShoot.java` deleted in Stage 5, replaced by `ShootWhenReady.java`. Whether the caching behavior survived into the replacement was not checked this pass. |

**The two most consequential gaps this stage claimed as done and demonstrably were not,**
found and fixed only in this session's `logging_plan.md`:
- `OnboardLogger.logAll()` — defined, never called anywhere. Every value any subsystem
  registered through it (including Stage 2's own claimed telemetry) was silently never
  written to the `.wpilog` file.
- `StatusSignalUtil.refreshAll()` — same pattern. Bulk-registered CAN signals (Shooter's
  and Turret's voltage/current, used for control math, not just logging) only updated at
  CTRE's slow default background rate instead of the 50 Hz loop rate.
- The drivetrain (8 motors) had **zero** electrical telemetry of any kind under Stage 2 or
  any stage before today — not claimed as done anywhere, but worth naming since Stage 2's
  title is "Logging & Telemetry Improvements" with no scope note excluding the drivetrain.

**Root cause pattern:** Stage 2's table records "added telemetry to Flywheel/Turret" as a
fact about the codebase, but does not carry forward when Stage 5 deleted `Flywheel.java`
and replaced `Turret.java` outright (Appendix B does note the deletions, the Stage 2 table
does not cross-reference them). A reader consulting only the Stage 2 section — exactly what
prompted this audit — reasonably concludes current telemetry is both complete and something
Stage 2 delivered, when neither is true.

### Stage 3 (Pre-Integration Cleanup) — Not independently re-checked this pass
Low risk (subtractive); overlaps with `cleanup.md` which gets its own audit stage (R-2).

### Stages 4–5 (Hackbots Integration) — Not independently re-checked this pass
High-risk stages by their own risk rating, but their claims (new files added, old files
deleted) are structurally easy to verify later via `git log`/file existence if needed —
deferred, not because they're assumed safe.

### Stage 6 (RobotContainer & Auto) — contradiction found and resolved: **`PROGRESS.md` is
right, the master plan's status table is stale.**

The master plan's own top-level status table (line 22) marks Stage 6 "⬜ Next," but
`PROGRESS.md` marks every Stage 6 line item ✅ under commit `stage-6`. Resolved by checking
both git history and current code:
- `git log` confirms commit `472c654` — `"stage-6: RobotContainer & auto — NamedCommands, turret jog, aiming telemetry"`
- `RobotContainer.java` currently contains exactly what `PROGRESS.md` claims: `aimAndShoot()`
  and all ten `ShootFuel*`/`NewShootFuel*` `NamedCommands.registerCommand` calls

Stage 6 shipped; the master plan's status table (and by extension its dependency chain —
"Stage 7 | Dependency: Stage 6 verified") was simply never updated after the fact. Low risk
in substance, but exactly the kind of stale top-level claim that misleads anyone who reads
only the status table at the top of the file rather than the detailed sections.

### Stage 7 (Lead Compensation) — marked done in both files; not independently re-checked.
`PROGRESS.md`'s own architecture note says hardware verification (Stage 8) is still
required — consistent, not a contradiction.

### Stage 8 (Hardware) — correctly marked not-started in both files (blocked on hardware).

---

### R-1 Summary

| Finding class | Count |
|---|---|
| ✅ Verified, matches current code | 10 |
| ⚠️ Stale (target replaced/deleted, claim's substance partly or fully lost) | 4 |
| ❌ Contradicted outright | 1 |
| Cross-file contradiction, resolved (Stage 6 status — master plan's status table is stale, `PROGRESS.md` is correct) | 1 |
| Not independently checked this pass (Stages 1, 3, 4, 5, 7 in full; C-3/C-4 in Stage 0) | — |

The user's concern about Stage 2 is confirmed: "✅ Done" overstates what's actually true
today. Recommend the eventual fix-it plan (a) add supersession cross-references to every
per-stage table, not just Appendix B, and (b) update the master plan's top-level status
table to mark Stage 6 done, since `PROGRESS.md` and current code both confirm it shipped.

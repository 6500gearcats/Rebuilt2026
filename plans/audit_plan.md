# Rebuilt2026 — Plan Completion Audit

Verifies "done" claims in every existing plan/progress pair against the actual current
code, without making any changes. Triggered by discovering that Stage 2 of
`Rebuilt2026_RefactorPlan.md` ("Logging & Telemetry Improvements — ✅ Done") did not match
reality: `OnboardLogger.logAll()` and `StatusSignalUtil.refreshAll()` were both defined but
never called anywhere, and the drivetrain had zero per-motor electrical telemetry — found
and fixed in `logging_plan.md` (2026-09-09).

Track execution in `AUDIT_PROGRESS.md`. **Read-only work** — each stage produces findings,
not code changes. Findings feed a future fix-it plan once the audit is complete.

**Branch:** `leto`

---

## Method (applies to every stage below)

For each claimed "done" item:
1. Re-read the actual current file(s) named in the claim.
2. Confirm the specific behavior described still exists, unchanged, and functions as
   described (not just "a similar-looking line exists").
3. Note if the target file was later deleted/replaced by a subsequent stage — a claim can
   be truthful for its moment and still misleading to a reader today.
4. Classify each item: **Verified** / **Stale** (target since replaced or deleted, claim's
   substance may or may not survive elsewhere) / **Contradicted** (code does not match the
   claim) / **Not independently checked** (spot-check budget didn't reach it this pass).

## Order of Review

Chronological by each pair's own "last updated" date, which also happens to match
dependency order (later plans build on or clean up after earlier ones):

| Stage | Plan / Progress pair | Last updated | Why this position |
|---|---|---|---|
| R-1 | `Rebuilt2026_RefactorPlan.md` + `PROGRESS.md`, cross-referenced with `ISSUES.md` | 2026-09-05 | Root plan; `ISSUES.md` is its requirements source and has no progress file of its own — reviewed together |
| R-2 | `cleanup.md` + `CLEANUP_PROGRESS.md` | 2026-09-05 | Directly follows the master plan's Stage 5–7 work |
| R-3 | `pkg_plan.md` + `PKG_PROGRESS.md` | 2026-09-08 | Independent mechanical refactor, done after cleanup |
| R-4 | `sim_plan.md` + `SIM_PROGRESS.md` | 2026-09-08 | Independent sim-quality work; this session already found sim issues (Vision loop overruns, wireframe rendering, CAN ID collisions) it didn't cover — worth checking what it claims vs. what was actually found later |
| R-5 | `doc_plan.md` + `DOC_PROGRESS.md` | 2026-09-08 | Documentation pass, lower functional risk |
| R-6 | `logging_plan.md` + `LOGGING_PROGRESS.md` | 2026-09-09 | Written and executed live this session — lightest touch, mostly a confirmation pass since every claim was verified by compiling as it was written |

---

## R-1 — Master Plan + ISSUES.md

Scope: all 9 stages (0–8) in `Rebuilt2026_RefactorPlan.md`, with particular depth on
**Stage 2 (Logging & Telemetry)** per the specific concern that triggered this audit.
Cross-reference Appendix A's issue-resolution map against `ISSUES.md`'s own severity list.

Known risk going in: several Stage 0–2 fixes targeted files (`Flywheel.java`, old
`Turret.java`, `AlignTurretToHub.java`, `UpToSpeedHopperShoot.java`, `CoolSnurbo.java`) that
Stage 5 later deleted or wholesale-replaced. Appendix A and Appendix B both note some of
these supersessions — the per-stage tables (Stage 1, Stage 2) mostly do not. Whether that's
just a presentation gap or hides a real regression needs checking per item.

---

## R-2 — Cleanup Plan

Scope: C-1 through C-5 in `cleanup.md`. All were marked done in one commit
(`cleanup-c1-c4`); C-5 is intentionally deferred to Stage 8. Verify the dead-code removals
(`targetPose`, `getBestPoseTarget()`, unused `SwerveRequest` fields, `LedCANdle` instance)
actually happened and nothing since reintroduced them.

---

## R-3 — Package Reorg Plan

Scope: P-1 through P-10 in `pkg_plan.md`. **Already-spotted inconsistency to resolve
first:** `PKG_PROGRESS.md`'s own task table marks P-1 through P-4 as ⬜ (not started), but
its Summary table below claims "P-1 — P-7 (initial consolidation) ✅". These two claims
contradict each other in the same file — establish which is true against the actual
package layout before evaluating anything else in this stage.

---

## R-4 — Simulation Plan

Scope: S-1 through S-6 in `sim_plan.md`. S-2 (PhotonVision deprecations) and S-6 (auto
path validation) are already marked incomplete in `SIM_PROGRESS.md` — confirm that's still
accurate. For S-3/S-4/S-5 (sim state seeding for Shooter/Hopper/Intake), verify against
current file contents rather than assume the commit did what its message says.

---

## R-5 — Documentation Plan

Scope: D-1 through D-3 in `doc_plan.md`. Lower risk — spot-check a sample of files per
priority tier rather than re-reading all ~20 touched files line by line, since Javadoc
presence/absence is easy to confirm quickly and low-consequence if something slipped.

---

## R-6 — Logging Plan

Scope: L-0 through L-6 in `logging_plan.md`. This was authored and executed within the
current session with a compile check after every stage, so treat this as a confirmation
pass: re-grep for the specific call sites (`OnboardLogger.logAll()`,
`StatusSignalUtil.refreshAll()`, the 16 `registerEnergy` calls) rather than a full re-audit.

---

## Output

Each stage's findings get appended to `AUDIT_PROGRESS.md` under that stage's heading, then
this file's per-stage status is marked done. Once all six stages are reviewed, findings
across all of them get consolidated into a new fix-it plan (not written yet).

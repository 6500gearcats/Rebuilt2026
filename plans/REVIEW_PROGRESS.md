# Rebuilt2026 — Code Review Remediation Progress

Tracks execution of `review_plan.md`. See that file for evidence, fix detail, and
verification steps for every task below.

**Branch:** `leto`
**Last updated:** 2026-09-09 (Stages R1 and R2 complete — 4 commits, all compiled clean;
paused before R3 pending user input, see note at that stage)

---

## Legend
- ✅ Done (committed — record the hash)
- 🔄 In progress
- ⬜ Not started
- ⏸ Blocked on a decision (see decision log)
- ⏭ Deliberately skipped (record why)

---

## Decision log

Resolve these before starting the tasks they gate. Record the answer, the date, and who
decided — an undocumented reversal is exactly what went wrong with `SIM_PROGRESS.md` S2-2.

| # | Gates | Question | Decision | Date / by |
|---|---|---|---|---|
| D-1 | R1-A1 | Which AprilTag layout is authoritative — `k2026RebuiltAndymark` or `kDefaultField`? Also: does the PhotonVision coprocessor's own configured layout match? | ✅ **Use `k2026RebuiltAndymark`.** All consumers reference it. Do **not** delete the `kDefaultField` alternative — leave it in place, documented in a comment explaining what it is and why it isn't used. | 2026-09-09 / james |
| D-2 | R1-A3 | Wire `Telemetry` up, or delete it? | ✅ **Wire it up.** Also fix the wrong `DriveState/Pose` instruction introduced in the earlier `README.md` + `configureLogging()` Javadoc commits. | 2026-09-09 / james |
| D-3 | R4 C-2, C-3 | Are `StaggerHopper` / `ControllerRumble` / `RunHopper` staged for future bindings, or abandoned? | ✅ **Leave in place for now.** Mark each with a `TODO` noting it is currently unreferenced and should be evaluated and deleted if it stays unused. | 2026-09-09 / james |
| D-4 | R5-7 | Energy reset semantics — per-enable, per-match, or cumulative since boot? | ✅ **Per-enable, and make it configurable.** Per-enable is the default; the reset policy is selectable per registration. | 2026-09-09 / james |
| D-5 | R4 | Keep `SysIDUtil` placeholders? (`cleanup.md` C-5 already said keep for Stage 8 — reconfirm only) | ✅ **Keep for now.** Reconfirms `cleanup.md` C-5. | 2026-09-09 / james |

> **Interpretation note on D-1 — flag if wrong.** "Don't delete the other" is being implemented
> as: keep the `VisionConstants.kTagLayout` *constant* (four files import it, so removing the
> name would churn imports), repoint its value at the Andymark layout so every consumer
> agrees, and document the previous `kDefaultField` value in an adjacent comment explaining
> what it was and why it changed. Net effect: one authoritative layout at runtime, the
> alternative preserved as documentation rather than as a second live constant.

---

## Stage R1 — Correctness bugs

**Do first.** Everything downstream assumes pose is correct.

| Task | Description | Severity | Status | Commit |
|------|-------------|----------|--------|--------|
| R1-A1 | Point all consumers at `k2026RebuiltAndymark`; document the `kDefaultField` alternative in a comment (D-1) | HIGH | ✅ | `923c17a` |
| R1-A2 | Fix `Vision`'s `SwerveDrivePoseEstimator` kinematics (12.75" assumed vs 13.5" actual) | HIGH | ✅ | `7b1a934` |
| R1-A3 | **Wire** `Telemetry` up (D-2); correct `README.md` + `configureLogging()` Javadoc | MED | ✅ | `e1b988d` |

**R1-A3 carries a documentation-correction obligation regardless of D-2's outcome** — the
`DriveState/Pose` instruction in `README.md` is wrong today and was published 2026-09-09.

**Stage R1 complete as of `e1b988d`.** One follow-up is not yet closed: R1-A3's loop-timing
concern (CTRE calls the telemetry consumer from its 100 Hz odometry thread, not the 50 Hz
robot loop) is documented in code but **not empirically verified** — needs one sim run
checking Tracer output for overruns or NT4 flood. Flagging here so it isn't lost.

---

## Stage R2 — Aim pipeline caching

| Task | Description | Severity | Status | Commit |
|------|-------------|----------|--------|--------|
| R2-B1/B2 | Cache `getAimParams()` once per loop in `RobotStateMachine` | HIGH | ✅ | `5163617` |
| R2-B3 | Remove per-call `Trigger` allocation in `isShootReady()` / `Shooter.tracked()` / `Turret.tracked()` | LOW | ✅ | `5163617` |

**Open question resolved during implementation:** the caching does **not** introduce new
staleness. `turretPose` (the input to `computeAimParams()`) was already refreshed only once
per loop in `periodic()`, which itself runs after `CommandScheduler.run()` — so commands
executing during the scheduler pass were already reading the previous loop's `turretPose`
before this change. The cache refresh stayed in its natural spot in `periodic()`; nothing
moved in `Robot.robotPeriodic()`.

**Scope correction found during implementation:** `Turret.tracked()` has zero callers
anywhere (only `Shooter.tracked()` is invoked, via `isShootReady()`). Its extraction is
preventative symmetry, not a measured fix. Its Javadoc also incorrectly claimed
`isShootReady()` used it — corrected.

**Stage R2 complete as of `5163617`.**

---

## Stage R3 — Side-effect getters — ⏭ **Deferred 2026-09-09 / james**

Reason: hygiene, not an active failure; R3-B4 touches FMS-timing-dependent LED/scoring-window
behavior that is genuinely hard to verify off-field. Revisit after competition, or sooner if
the schedule allows. R1 and R2 landed regardless — this defer does not block R4 onward, none
of which depend on R3.

| Task | Description | Severity | Status | Commit |
|------|-------------|----------|--------|--------|
| R3-B4 | Split `getState()` into `updateStateMachine()` (mutating, called once from `periodic()`) + a pure `getState()` | MED | ⏭ deferred | |
| R3-B5 | De-duplicate `refreshPoseFromVision()` (currently 2–3× per loop) | LOW | ⏭ deferred | |

---

## Stage R4 — Dead code sweep — ✅ Complete as of `efc9f05`

| Task | Item | Status | Commit |
|------|------|--------|--------|
| C-1 | `Telemetry` | ⏭ **not deleted** — D-2 chose to wire it up instead; handled in R1-A3 | |
| C-2 | `StaggerHopper`, `ControllerRumble` — TODO added, kept (D-3) | ✅ | `efc9f05` |
| C-3 | `RunHopper` — TODO added, kept (D-3) | ✅ | `efc9f05` |
| C-4 | `ShooterValuesSenable` — deleted | ✅ | `efc9f05` |
| C-5 | `getEstimationStdDevs()` ×2 + orphaned `kSingleTagStdDevs`/`kMultiTagStdDevs` — deleted | ✅ | `efc9f05` |
| C-6 | `Vision.gccPub` / `gcdPub` — deleted | ✅ | `efc9f05` |
| C-7 | 5 dead `RobotStateMachine` methods — deleted; `underTrench()` geometry preserved in a comment | ✅ | `efc9f05` |
| C-8 | `RobotContainer.photonVisionIO` — deleted | ✅ | `efc9f05` |
| C-9 | `RobotContainer` × 3 `SlewRateLimiter` — deleted | ✅ | `efc9f05` |
| C-10 | **Expanded during verification** — deleted 6 entire nested classes (`DriveConstants`, `ModuleConstants`, `OIConstants`, `AutoConstants`, `NeoMotorConstants`, `GyroConstants`), all zero external references, not just the originally-catalogued subset | ✅ | `efc9f05` |
| C-11 | `SmartDashboard.putNumber("Shoot Speed", 0)` — deleted | ✅ | `efc9f05` |
| C-12 | `kRobotToCam` — deleted (was unused, and had the radians/degrees bug) | ✅ | `efc9f05` |
| C-13 | **Corrected during verification** — `ProjectHailMaryRight.auto` *does* reference `SpeedUp`. Implemented as `m_shooter.shoot(m_stateManager::aimParams)` rather than removed | ✅ | `efc9f05` |

### Two findings during execution, beyond the original review

**AutoConstants never executed, ever.** Deleting it (part of C-10's scope expansion) surfaced
that this class held its own independent "load `RobotConfig` safely, report failure"
implementation — the exact fix `ISSUES.md`/the master plan record as **M-3, done in Stage
0**. But nothing in the codebase ever referenced the `AutoConstants` class, and Java only
runs a class's static initializer on first reference — so that try/catch never ran, not
once, in this robot's entire operating history. The actually-reachable equivalent lives in
`CommandSwerveDrivetrain.configureAutoBuilder()`. M-3's fix is real; the master plan's
tracker just pointed at the wrong (dead) copy of it.

**This is also a correction to this session's own earlier audit.** `AUDIT_PROGRESS.md` R-1
verified M-3 as "✅ Verified" by confirming the code text matched the claimed fix — it did
not check whether that code path was ever reachable. Text-matches-claim and
code-path-executes are different questions; this pass only checked the first. Noted in
`AUDIT_PROGRESS.md` directly.

**`SpeedUp` was a real auto-behavior gap, not dead code.** `ProjectHailMaryRight.auto` runs
it in a `deadline` block with the `PHM1` path and `Intake`, immediately before the first
`ShootFuel3s` — clearly meant to warm up the flywheel while driving/intaking. As
`Commands.none()`, that auto has been shooting cold every time it runs. Implemented, not
removed.

---

## Stage R5 — Logging additions

**Do after R2**, so new suppliers aren't layered on an uncached pipeline.

| Task | Addition | Value | Status | Commit |
|------|----------|-------|--------|--------|
| R5-1 | Command lifecycle hooks (`onCommandInitialize` / `Finish` / `Interrupt`) | ★ highest | ⬜ | |
| R5-2 | `PowerDistribution` — total/per-channel current, voltage, temperature | high | ⬜ | |
| R5-3 | Match context — event, match type/number, alliance, FMS attached (one-shot) | high | ⬜ | |
| R5-4 | Loop timing to `.wpilog` | high | ⬜ | |
| R5-5 | Robot health — brownout, battery to `.wpilog`, full `CANStatus` error counters | med | ⬜ | |
| R5-6 | Vision diagnostics — tag count, ambiguity, rejected-measurement counter | med | ⬜ | |
| R5-7 | Energy reset — **configurable policy, default per-enable** (D-4) — plus whole-robot aggregate | med | ⬜ ready | |

---

## Stage R6 — Tests

| Task | Target | Status | Commit |
|------|--------|--------|--------|
| R6-1 | `Turret.findCC()` — wrap-around edge cases (seam already exists, never used) | ⬜ | |
| R6-2 | `ToFAim.update()` — convergence, out-of-range, constraint violation | ⬜ | |
| R6-3 | `LeadCompensator.computeLeadTarget()` — zero velocity, lead direction, impossible inner strategy | ⬜ | |
| R6-4 | `RobotStateMachine.checkZone()` — zone boundaries, both alliances | ⬜ | |
| R6-5 | Delete/rename empty `AlignTest.java` (named after a deleted command) | ⬜ | |

---

## Summary

| Stage | Tasks | Status |
|-------|-------|--------|
| R1 — Correctness | 3 | ⬜ |
| R2 — Aim caching | 2 | ⬜ |
| R3 — Side-effect getters | 2 | ⬜ |
| R4 — Dead code | 13 | ⬜ |
| R5 — Logging | 7 | ⬜ |
| R6 — Tests | 5 | ⬜ |

---

## Provenance note

Two findings in this plan were introduced by the AI-assisted logging work earlier the same
day (2026-09-09), not by prior team development:

- **R2-B1** — wiring `OnboardLogger.logAll()` into `robotPeriodic()` activated seven dormant
  `AimParams.setupLogging` suppliers, each running a full aiming-pipeline evaluation every
  loop.
- **R1-A3** (documentation half) — the `DriveState/Pose` instruction given during sim
  debugging and then written into `README.md` was wrong, because `Telemetry` is never
  instantiated and that topic does not exist.

Recorded here so the history is legible rather than implied.

---

## Findings during execution

*(Append anything discovered while implementing that wasn't in the original review — the
CAN ID collision found during the README rewrite is the model for this: a real bug surfaced
by doing adjacent work carefully.)*

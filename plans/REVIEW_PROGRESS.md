# Rebuilt2026 — Code Review Remediation Progress

Tracks execution of `review_plan.md`. See that file for evidence, fix detail, and
verification steps for every task below.

**Branch:** `leto`
**Last updated:** 2026-09-09 (all 5 decisions resolved; no tasks started — R1 is ready to begin)

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
| R1-A1 | Point all consumers at `k2026RebuiltAndymark`; document the `kDefaultField` alternative in a comment (D-1) | HIGH | ⬜ ready | |
| R1-A2 | Fix `Vision`'s `SwerveDrivePoseEstimator` kinematics (12.75" assumed vs 13.5" actual) | HIGH | ⬜ ready | |
| R1-A3 | **Wire** `Telemetry` up (D-2); correct `README.md` + `configureLogging()` Javadoc | MED | ⬜ ready | |

**R1-A3 carries a documentation-correction obligation regardless of D-2's outcome** — the
`DriveState/Pose` instruction in `README.md` is wrong today and was published 2026-09-09.

---

## Stage R2 — Aim pipeline caching

| Task | Description | Severity | Status | Commit |
|------|-------------|----------|--------|--------|
| R2-B1/B2 | Cache `getAimParams()` once per loop in `RobotStateMachine` | HIGH | ⬜ | |
| R2-B3 | Remove per-call `Trigger` allocation in `isShootReady()` / `Shooter.tracked()` / `Turret.tracked()` | LOW | ⬜ | |

**Open question inside R2-B1** (decide during implementation, record the answer here): the
cache refreshes in `RobotStateMachine.periodic()`, which runs *after* `CommandScheduler.run()`
in `Robot.robotPeriodic()`. Commands would therefore read a value one loop (20 ms) stale —
~10 cm of travel at 5 m/s. Either accept and document it, or move the refresh ahead of
`CommandScheduler.run()`.

---

## Stage R3 — Side-effect getters

**Consider deferring if the competition schedule is tight** — hygiene, not active failure,
and it touches FMS-timing-dependent behavior that is hard to verify off-field.

| Task | Description | Severity | Status | Commit |
|------|-------------|----------|--------|--------|
| R3-B4 | Split `getState()` into `updateStateMachine()` (mutating, called once from `periodic()`) + a pure `getState()` | MED | ⬜ | |
| R3-B5 | De-duplicate `refreshPoseFromVision()` (currently 2–3× per loop) | LOW | ⬜ | |

---

## Stage R4 — Dead code sweep

| Task | Item | Status | Commit |
|------|------|--------|--------|
| C-1 | `Telemetry` | ⏭ **not deleted** — D-2 chose to wire it up instead; handled in R1-A3 | |
| C-2 | `StaggerHopper`, `ControllerRumble` — **add TODO, do not delete** (D-3) | ⬜ ready | |
| C-3 | `RunHopper` — **add TODO, do not delete** (D-3). Note it is reachable only via `StaggerHopper`, itself unreferenced | ⬜ ready | |
| C-4 | `ShooterValuesSenable` | ⬜ | |
| C-5 | `getEstimationStdDevs()` ×2 — **or** wire it up instead of deleting | ⬜ | |
| C-6 | `Vision.gccPub` / `gcdPub` (Limelight-era camera names) | ⬜ | |
| C-7 | `RobotStateMachine`: `getTurretPose()`, `isFarEnough()`, `setCurrentZone()`, `switchState()`, `underTrench()` — **preserve `underTrench()`'s field geometry in a comment first** | ⬜ | |
| C-8 | `RobotContainer.photonVisionIO` (permanently null) | ⬜ | |
| C-9 | `RobotContainer` × 3 `SlewRateLimiter` | ⬜ | |
| C-10 | `Constants`: `kTiltPitch`, `NeoMotorConstants`, `AutoConstants.config`, `DriveConstants` CAN IDs — **not `kDriveKinematics` until R1-A2 lands** | ⬜ | |
| C-11 | `SmartDashboard.putNumber("Shoot Speed", 0)` | ⬜ | |
| C-12 | `kRobotToCam` — delete, or fix the radians/degrees bug (`Rotation3d(0,0,180)`) | ⬜ | |
| C-13 | `NamedCommands("SpeedUp")` stub — implement or remove + confirm no `.auto` references it | ⬜ | |

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

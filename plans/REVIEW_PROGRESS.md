# Rebuilt2026 — Code Review Remediation Progress

Tracks execution of `review_plan.md`. See that file for evidence, fix detail, and
verification steps for every task below.

**Branch:** `leto`
**Last updated:** 2026-09-09 (Stages R1, R2, R4, R5, R6 complete. R3 deferred by decision.
Plan substantively done pending D-6 and the R1-A3 loop-timing verification below.)

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
| D-6 | `RobotStateMachine.checkZone()` / `FieldZone` enum | Is `checkZone()`'s Y-boundary logic wrong, or is the `FieldZone` enum's Javadoc wrong? They're exactly swapped (`y > 4.2` returns `NEUTRAL_BOTTOM` but the enum documents that range as `NEUTRAL_TOP`, and vice versa) | ⏸ *pending* — found 2026-09-09 while writing R6-4; needs the real field diagram/orientation to resolve, not inferable from code | |

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

## Stage R5 — Logging additions — ✅ Complete as of `a5d32d4`

Every API used (`PowerDistribution`, `DriverStation` match-context methods,
`RobotController.getCANStatus()`/`CANStatus` fields, `CommandScheduler.onCommand*` hooks,
`PhotonTrackedTarget.getPoseAmbiguity()`, `IntegerLogEntry`/`BooleanLogEntry` constructors)
was verified against WPILib/Phoenix6/PhotonLib sources before use, not assumed.

| Task | Addition | Value | Status | Commit |
|------|----------|-------|--------|--------|
| R5-1 | Command lifecycle hooks — 3 `StringLogEntry` channels (Initialized/Finished/Interrupted), direct `DataLog` writes (not `OnboardLogger` — discrete events don't fit its poll model) | ★ highest | ✅ | `a5d32d4` |
| R5-2 | `PowerDistribution` — total/per-channel current, total power/energy, voltage, temperature, via a new `"Robot"` `OnboardLogger` | high | ✅ | `a5d32d4` |
| R5-3 | Match context — event/match type/number/replay/alliance/FMS-attached, one-shot on first DS attach via a guard flag | high | ✅ | `a5d32d4` |
| R5-4 | `Robot/LoopTimeSec` — wall-clock delta between successive `robotPeriodic()` calls, measured first thing in the method | high | ✅ | `a5d32d4` |
| R5-5 | Brownout, battery voltage (now in `.wpilog` too, not just 10 Hz SmartDashboard), full `CANStatus` (`busOffCount`/`txFullCount`/`receiveErrorCount`/`transmitErrorCount`) | med | ✅ | `a5d32d4` |
| R5-6 | Vision diagnostics — added `getTagCount()`/`getBestTargetAmbiguity()` to `VisionIO`, implemented in both implementers; per-camera logging + cumulative `Vision/RejectedMeasurementCount` | med | ✅ | `a5d32d4` |
| R5-7 | `OnboardLogger.EnergyReset` enum (`NEVER`/`ON_ENABLE`), 4-arg overload, 3-arg defaults to `ON_ENABLE` — all 16 existing call sites pick this up unchanged. Static `getTotalEnergyJ()`/`getTotalPowerW()` aggregate across every registration process-wide, wired to `Robot/EnergyJ`/`Robot/PowerW` for cross-check against the PDH's independent total | med | ✅ | `a5d32d4` |

**R5-4 note:** explicitly documented as a plain field measurement, not a WPILib Tracer epoch —
avoids repeating the "`<Subsystem>.periodic()` epoch also covers `simulationPeriodic()`"
confusion this session hit earlier (see the `project_sim-loop-overruns` memory).

---

## Stage R6 — Tests — ✅ Complete

16 tests across 4 files, all passing. Every expected value was hand-traced against the
actual algorithm *before* being written as an assertion, then verified empirically by
running the suite — not derived from what the code happened to return.

| Task | Target | Status | Commit |
|------|--------|--------|--------|
| R6-1 | `Turret.findCC()` — 6 tests: no-wrap, small adjustment, wrap positive/negative, clamp-when-unreachable, exact-0.5-boundary | ✅ | |
| R6-2 | `ToFAim.update()` — 3 tests: stationary direct lookup, out-of-range distance clamped then constraint-rejected, in-range shot still constraint-rejected | ✅ | |
| R6-3 | `LeadCompensator.computeLeadTarget()` — 3 tests: zero velocity, lead shift opposite travel direction, impossible inner strategy breaks cleanly | ✅ | |
| R6-4 | `RobotStateMachine.checkZone()` — 4 tests: Blue/Red alliance boundaries, neutral-zone Y bands. Required `HAL.initialize()` + `DriverStationSim` (both verified against WPILib 2026.2.1 sources) since the singleton reads live `DriverStation.getAlliance()` | ✅ | |
| R6-5 | Deleted empty `AlignTest.java` (named after `AlignTurretToHub`, deleted Stage 5) | ✅ | |

### Real finding, surfaced while writing R6-4 — needs a human decision

`checkZone()`'s Y-boundary code and the `FieldZone` enum's own Javadoc **directly
contradict each other**:

| Y value | `checkZone()` returns | Enum Javadoc says this range means |
|---|---|---|
| `y > 4.2` | `NEUTRAL_BOTTOM` | `NEUTRAL_TOP` |
| `y < 3.8` | `NEUTRAL_TOP` | `NEUTRAL_BOTTOM` |

Exactly swapped. This can't be resolved from code alone — it requires knowing the actual
field orientation (which physical side is "top" on the field diagram this was written
against). The test (`RobotStateMachineTest.neutralZoneYBoundaries_currentBehaviorTopBottomSwappedVsEnumJavadoc`)
deliberately asserts what the code does today, not what's "correct" — its job is to
characterize existing behavior, and its name and Javadoc say so explicitly. **Needs a
decision: is `checkZone()`'s logic wrong, or is the enum's Javadoc wrong?** Whoever has the
field diagram/CAD in front of them can settle this in under a minute; I can't from here.

---

## Summary

| Stage | Tasks | Status |
|-------|-------|--------|
| R1 — Correctness | 3 | ✅ |
| R2 — Aim caching | 2 | ✅ |
| R3 — Side-effect getters | 2 | ⏭ Deferred |
| R4 — Dead code | 13 | ✅ |
| R5 — Logging | 7 | ✅ |
| R6 — Tests | 5 | ✅ |

**Open items remaining:** D-6 (the `FieldZone`/`checkZone()` swap — needs a human with the
field diagram) and the R1-A3 loop-timing empirical verification (one sim run watching for
overruns). Both are called out in their respective sections above; neither blocks anything
else in this plan.

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

Discovered while implementing this plan, not part of the original review:

- **R4-C10 scope expansion** — six entire `Constants.java` nested classes were dead
  (`DriveConstants`, `ModuleConstants`, `OIConstants`, `AutoConstants`, `NeoMotorConstants`,
  `GyroConstants`), not just the specific fields originally catalogued.
- **`AutoConstants` never executed, ever** — its `RobotConfig`-loading safety net (credited
  as Stage 0's M-3 fix) never ran because nothing referenced the containing class. Corrected
  the earlier `AUDIT_PROGRESS.md` R-1 verification of M-3, which had only checked that code
  text matched the claim, not that the code path was reachable.
- **R4-C13 scope correction** — `ProjectHailMaryRight.auto` actually references `SpeedUp`,
  contradicting the original review's assumption. It was `Commands.none()`; implemented as a
  real flywheel warm-up.
- **D-6 — `FieldZone`/`checkZone()` swap** (see above) — found while writing R6-4. Needs a
  human decision; can't be resolved from code alone.
- **Real crash, found running the sim after this session's changes** — `getAimParams()` threw
  `NullPointerException` on `this.constraints.check(params)` inside `ToFAim.update()`, taking
  the whole robot program down on the first loop. Root cause: `RobotStateMachine.instance`
  was declared *before* `kScoringConstraints` in the source. Java initializes static fields in
  textual order, and `instance`'s eager `new RobotStateMachine()` runs every instance field
  initializer — including `m_tofAim = new ToFAim(..., kScoringConstraints)` — before
  `kScoringConstraints`'s own initializer has run, so `m_tofAim` permanently captured `null`.
  This bug predates this session; it was latent because nothing called `getAimParams()`
  reliably until `OnboardLogger.logAll()` was wired up (`03368ea`), and R2's caching change
  moved the call into the unconditional per-loop path, making the crash immediate and certain
  rather than depending on the gunner holding the aim trigger. Fixed by reordering the two
  field declarations, with a Javadoc on `kScoringConstraints` explaining why the order
  matters. Added a regression test (`RobotStateMachineTest.getAimParamsDoesNotThrow`) — none
  of the R6-4 `checkZone()` tests exercised this path, since `checkZone()` never touches
  `m_tofAim`.

This list, plus the CAN ID collision found during the README rewrite (a separate plan), is
the running argument for why "verify by doing the adjacent work carefully" keeps finding
real bugs that a read-through alone would miss.

- **AdvantageScope deprecation notice, reported by the user 2026-09-09 (not a code review
  finding — external tooling lifecycle):** "The legacy numeric array format for structured
  data is deprecated and will be removed in 2027." Two sources confirmed in this codebase:
  - `Telemetry.java:63-64,120-121` — `fieldPub`/`fieldTypePub` publish robot pose as a raw
    `double[3]` (x, y, rotation degrees) under `Pose/robotPose` with a `Pose/.type =
    "Field2d"` marker. **This is pure redundancy** — the same class already publishes the
    identical pose as a modern struct at `DriveState/Pose` (line 47/107,
    `StructPublisher<Pose2d>`). Deleting the legacy pair loses no capability.
  - `Vision.java:104` — `public Field2d m_field = new Field2d();`, a WPILib-built-in
    `Sendable`. Its wire format is WPILib's own implementation detail, not something this
    codebase controls directly; migrating it means either WPILib updates `Field2d` itself, or
    this code stops relying on the `Field2d` widget and points AdvantageScope at a struct
    topic instead (already available: `DriveState/Pose`).
  - **`Telemetry.java`'s legacy pair actioned same day:** deleted `fieldPub`/`fieldTypePub`/
    `m_poseArray` and the `Pose` `NetworkTable` handle that only existed for them. Zero
    capability lost — `drivePose`/`DriveState/Pose` already covered the same data. Compiles
    clean, all 17 tests still pass. `Vision.m_field`/`Field2d` intentionally left alone —
    still useful for Shuffleboard/Elastic driver-station widgets (a different audience than
    AdvantageScope); `DriveState/Pose` remains the AdvantageScope-side source of truth.

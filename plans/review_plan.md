# Rebuilt2026 — Code Review Remediation Plan

Addresses the findings from the full-codebase review conducted 2026-09-09. Six stages,
ordered by risk and dependency. Track execution in `REVIEW_PROGRESS.md`.

**Branch:** `leto`
**Source:** review report delivered in-session 2026-09-09; every finding below was verified
against source at that time, with the file and method named so the reference survives line
drift.

---

## Ground rules (per `PLANNING_GUIDE.md`)

- Update `REVIEW_PROGRESS.md` in the **same commit** as the code change it describes.
- Cite the commit hash in the progress tracker when each task lands.
- If the real implementation diverges from a task's shape below, **edit this plan** rather
  than leaving a stale checkbox — the mismatch between plan shape and commit shape is
  exactly what caused `PKG_PROGRESS.md` to drift.
- Compile after every stage: `$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"` then
  `.\gradlew.bat compileJava`.
- Two findings in Stage R1/R2 (**A3** and **B1**) were introduced or worsened earlier the
  same day by the logging work; they are not pre-existing team defects.

---

## ⚠ Decision points — resolve before starting the affected task

These need a human call. Do not guess; record the answer in `REVIEW_PROGRESS.md`'s decision
log with the date and who decided.

| # | Task | Question | Why it can't be inferred from code |
|---|---|---|---|
| D-1 | R1-A1 | Which AprilTag layout is authoritative — `k2026RebuiltAndymark` or `kDefaultField`? | Depends on which physical field variant the team competes on. Both are currently loaded; the code can't tell which is intended. **Also confirm the PhotonVision coprocessor's own configured layout matches** — that's a third source of truth living outside this repo. |
| D-2 | R1-A3 | Wire `Telemetry` up, or delete it? | Wiring restores `DriveState/*` topics (module states, targets, speeds, odometry frequency — useful in AdvantageScope). Deleting removes ~130 lines of unused code. Either is defensible; the current half-state is not. |
| D-3 | R4 | Are `StaggerHopper` / `ControllerRumble` / `RunHopper` intended for future use, or genuinely abandoned? | All three are complete, documented, working commands with zero call sites. They may be staged for teleop bindings not yet written. |
| D-4 | R5-E7 | Should accumulated energy reset per-enable, per-match, or stay cumulative since boot? | Product decision about how the numbers get read. Affects `OnboardLogger.registerEnergy`'s contract. |
| D-5 | R4 | Keep `SysIDUtil` placeholders? | `cleanup.md` C-5 already decided "keep, deferred to Stage 8." Reconfirm rather than re-litigate. |

---

# Stage R1 — Correctness bugs

**Risk: MEDIUM.** Small diffs, but all three touch pose/aiming behavior. Do these first;
everything downstream reasons about pose being correct.

---

## R1-A1 — Unify the two AprilTag field layouts

**Severity: HIGH — systematic aiming error that vision cannot correct.**

### Evidence

| Constant | Value | Consumers |
|---|---|---|
| `Constants.APRIL_TAG_FIELD_LAYOUT` | `AprilTagFields.k2026RebuiltAndymark` | `RobotStateMachine.checkAlliance()` — sets `Tag_POSE2D` (tags 10/20) and therefore `HubPose`, the aim target |
| `Constants.VisionConstants.kTagLayout` | `AprilTagFields.kDefaultField` | `PhotonVisionIO` + `PhotonVisionSimIO` estimator constructors; `Vision.setUpSim()`; `getEstimationStdDevs()` |

The robot localizes against one map and aims at a hub derived from a different one. Both
halves are internally self-consistent, so the error is a **constant offset** — vision never
corrects it, and it will not appear as jitter or drift. It looks like "our math is right and
we still miss."

### Fix

1. Resolve **D-1** first.
2. Delete one constant. Keep a single `public static final AprilTagFieldLayout` (suggest
   keeping it in `Constants` at top level, since both vision and the state machine need it).
3. Update all consumers listed above to reference the surviving constant.
4. Add a one-time startup log line recording which layout is loaded (name + tag count), so
   the answer is visible in every `.wpilog` rather than requiring a source read.

### Verification

- `grep -rn "AprilTagFields\." src/` returns exactly **one** call site.
- Startup log shows the expected layout name.
- On hardware: place the robot a measured distance from the hub, compare
  `Robot/DistToHubM` against the tape measure. A layout mismatch shows up here.

---

## R1-A2 — Vision pose estimator uses the wrong swerve kinematics

**Severity: HIGH — corrupts odometry propagation feeding both aiming and PathPlanner.**

### Evidence

- `Vision.java`, constructor: `new SwerveDrivePoseEstimator(DriveConstants.kDriveKinematics, …)`
- `Constants.DriveConstants.kDriveKinematics` is built from
  `kTrackWidth = kWheelBase = Units.inchesToMeters(25.5)` → modules at **±12.75"**
- `TunerConstants2` `kFrontLeftXPos`/`YPos` etc. → actual modules at **±13.5"**

~6% error in effective track width and wheelbase. Rotation-derived translation from module
deltas is mis-scaled. This estimate becomes `RobotStateMachine.pose`, consumed by turret
aiming and by PathPlanner's `AutoBuilder` pose supplier (`RobotStateMachine::getPose`).
Vision measurements bound the error, so it presents as drift **between** vision updates,
worst when tags are not visible — e.g. mid-auto in a trench.

Aggravating factor: `DriveConstants`' own Javadoc calls the class legacy "for legacy code
compatibility," so a reader reasonably assumes nothing in it is live. One field of it is.

### Fix — preferred

Source kinematics from the drivetrain itself, eliminating the second definition:

```java
// CommandSwerveDrivetrain already exposes CTRE's real module locations.
// getModuleLocations() confirmed present in Phoenix 6 26.1.0 SwerveDrivetrain.java.
new SwerveDriveKinematics(drivetrain.getModuleLocations())
```

`Vision` takes this via constructor injection alongside its existing suppliers, rather than
importing `DriveConstants`.

### Fix — fallback

If injection is awkward, correct `DriveConstants.kTrackWidth`/`kWheelBase` to `27.0` inches
(2 × 13.5) **and** add a comment pointing at `TunerConstants2` as the authority. This leaves
two definitions that must be kept in sync — inferior, but a valid stopgap.

### Verification

- Log both `drivetrain.getModuleLocations()` and the estimator's kinematics at startup;
  confirm identical.
- In sim: drive a 360° in-place rotation, confirm the estimated pose returns to its start
  translation. A track-width error shows as translational creep during pure rotation.

---

## R1-A3 — `Telemetry` is never instantiated

**Severity: MEDIUM (functional) / HIGH (documentation harm — already published).**

### Evidence

`grep -rn "new Telemetry\|registerTelemetry\|telemeterize" src/` returns only the class's own
definition. Nothing constructs it; `telemeterize()` is never called. Every `DriveState/*`
NT topic it declares — `Pose`, `Speeds`, `ModuleStates`, `ModuleTargets`, `ModulePositions`,
`Timestamp`, `OdometryFrequency` — **does not exist at runtime.**

This is the fourth instance of this exact pattern in one session, after
`OnboardLogger.logAll()`, `StatusSignalUtil.refreshAll()`, and this.

### Downstream damage already shipped

1. During sim debugging I told the user to drag `DriveState/Pose` into AdvantageScope's 2D
   field view. That advice could never have worked.
2. The same wrong instruction was written into the new root **`README.md`**
   (Building & Deploying section) and into `CommandSwerveDrivetrain.configureLogging()`'s
   Javadoc, which claims kinematic state "was published, by `Telemetry`."

**Both must be corrected regardless of how D-2 is decided.**

What actually publishes pose today: `StateMachine/RobotPose` and `StateMachine/TurretPose`
(10 Hz, `RobotStateMachine.periodic()`), and `SmartDashboard/Field` (`Vision.m_field`).

### Fix

Resolve **D-2**, then:

- **If wiring up:** construct `Telemetry` in `RobotContainer` and register it —
  `drivetrain.registerTelemetry(telemetry::telemeterize)`. Confirm the CTRE
  `registerTelemetry` signature against Phoenix 6 26.1.0 before writing it. Note this adds
  per-loop NT publishing; consider whether it should be rate-limited given the loop-overrun
  history (Stage 0).
- **If deleting:** remove `Telemetry.java`, its 4 `Mechanism2d` widgets, and the
  `SmartDashboard.putData("Module N", …)` calls in its constructor.

Then, in **both** cases:
- Correct `README.md` → point at `StateMachine/RobotPose` (or whatever survives).
- Correct the `configureLogging()` Javadoc claim in `CommandSwerveDrivetrain`.

### Verification

- Launch sim, connect AdvantageScope, confirm the topic named in `README.md` actually exists
  in the NT tree. **Do not mark this task done on a compile alone** — the whole finding is
  that the code compiled fine while publishing nothing.

---

# Stage R2 — Aim pipeline caching

**Risk: LOW-MEDIUM.** Pure de-duplication; no math changes. Fixes a regression introduced
2026-09-09 plus pre-existing duplication.

---

## R2-B1/B2 — Cache `getAimParams()` once per loop

**Severity: HIGH (performance) — ~10 identical full-pipeline evaluations per 20 ms loop.**

### Evidence — call sites per loop while shooting

| Caller | Calls/loop | Notes |
|---|---|---|
| `AimParams.setupLogging` suppliers | **7** | Seven `registerX` entries, each supplier calls `params.get()` → `getAimParams()`. Dormant until `OnboardLogger.logAll()` was wired into `robotPeriodic()` on 2026-09-09 — **this is the introduced regression** |
| `Turret.track()` | 2 | Once via `state.aimParams()`; again via `state.shootReady.getAsBoolean()` → `isShootReady()` → `getAimParams()` |
| `Shooter.shoot()` | 1 | Via the `paramsSupplier` passed from `AimPrep` |
| `RobotStateMachine.periodic()` | 0.1 | `isShootReady()` at 10 Hz |

Each evaluation runs `LeadCompensator.computeLeadTarget` (up to 5 iterations, each invoking
`ToFAim.update`) plus a final `ToFAim.update`, allocating `Pose3d`, `Translation2d`, and
`AimParams` throughout. `ISSUES.md` C-3 rated "150+ short-lived objects/second" as
**Critical**; this adds roughly 7,000/second.

### Fix

Cache in `RobotStateMachine`:

```java
private AimParams m_cachedAimParams = AimParams.impossible();

// In periodic(), Tier 1 (every loop), AFTER pose/turretPose are updated:
m_cachedAimParams = computeAimParams();   // the current body of getAimParams()

// Public accessor becomes a field read:
public AimParams getAimParams() {
    return m_cachedAimParams;
}
```

Rename the existing body to a private `computeAimParams()`. Every existing consumer keeps
calling `getAimParams()` unchanged and transparently gets the cached value.

**Ordering requirement:** the cache must be refreshed in `periodic()` *after* `pose` and
`turretPose` are updated, and `RobotStateMachine.periodic()` is called from
`Robot.robotPeriodic()` **after** `CommandScheduler.run()`. That means commands executing
this loop read the value computed at the *end of the previous* loop — one cycle (20 ms) of
staleness. At 5 m/s that's 10 cm of robot travel. **Evaluate whether this matters**; if it
does, move the cache refresh into `Robot.robotPeriodic()` before `CommandScheduler.run()`,
or accept it and document it explicitly.

### Verification

- Add a temporary invocation counter in `computeAimParams()`; log per loop; confirm it reads
  exactly 1. Remove the counter before committing.
- Compare `Vision.periodic()` / overall loop timing in the sim Tracer output before and after.
- Confirm `Aiming/*` values in the `.wpilog` are unchanged in value (only in cost).

---

## R2-B3 — `isShootReady()` allocates a `Trigger` per call

**Severity: LOW.** Not a registration leak — an unbound `Trigger` is never polled by the
`EventLoop` — but it is one object plus a lambda allocated on a path hit every loop.

`RobotStateMachine.isShootReady()` is `return m_Shooter.tracked(() -> getAimParams()).getAsBoolean();`
and `Shooter.tracked()` does `return new Trigger(...)`.

### Fix

Extract the predicate from `Shooter.tracked()` into a plain
`boolean isTracked(AimParams params)` method; have `tracked()` wrap that in a `Trigger` for
binding use, and have `isShootReady()` call `isTracked(getAimParams())` directly with no
allocation. Same treatment applies to `Turret.tracked()`.

### Verification
Compile; confirm `StateManager.shootReady` (the one genuinely-bound `Trigger`) still works —
`Turret.track()` reads it every loop to select coarse vs. tight tolerance.

---

# Stage R3 — Side-effect getters

**Risk: MEDIUM-HIGH.** Touches the LED/scoring-window state machine, which is timing- and
FMS-dependent and hard to test off-field. Consider deferring until after competition if the
schedule is tight — these are hygiene issues, not active failures.

## R3-B4 — `getState()` mutates state and writes NT on every call

`RobotStateMachine.getState()` calls `setState()` (→ `refreshPoseFromVision()` →
`update()`), mutates `switching` / `switchingRed` / `switchingGreen` / `exampleColor`, and
performs an unconditional `SmartDashboard.putNumber("Robot/TimeUntilSwitchSec", …)`. Any
caller triggers all of it. `isActive()` calls it — so does anything calling `isActive()`.

Its own Javadoc already warns about this ("**Warning — side effects**"); it was documented
during the D-1 doc pass rather than fixed.

### Fix
Split into two methods:
- `updateStateMachine()` — the current body, called **once** from `periodic()`. Keeps all
  mutation and the NT write.
- `getState()` — returns the `state` field with no side effects.

Audit every current `getState()` / `isActive()` caller to confirm none depended on the
side effect to drive a transition. **This is the risk**: if any caller implicitly relied on
`getState()` advancing the schedule, moving the advance to `periodic()` changes timing.

### Verification
In sim, drive the match clock through a full window schedule and confirm ACTIVE/INACTIVE
transitions and LED flag sequences occur at identical times before and after.

## R3-B5 — `refreshPoseFromVision()` runs 2–3× per loop

Called from `periodic()`, from `getPose()` (which PathPlanner's pose supplier hits every
loop during autos), and from `setState()` on transitions. Each call reads
`m_vision.getEstimatedPose()`.

### Fix
Refresh once in `periodic()`; make `getPose()` a plain field read. Lower value than B4 —
`getEstimatedPose()` is a cheap field read on the estimator — so bundle it with B4 or skip.

---

# Stage R4 — Dead code sweep

**Risk: LOW.** Purely subtractive. Model this on `cleanup.md`, which audited clean.
Resolve **D-3** and **D-5** first.

| # | Item | Location | Notes |
|---|---|---|---|
| C-1 | `Telemetry` | `subsystems/drivetrain/Telemetry.java` | Only if D-2 chose deletion; otherwise handled in R1-A3 |
| C-2 | `StaggerHopper`, `ControllerRumble` | `commands/` | Zero call sites. Gated on D-3 |
| C-3 | `RunHopper` | `commands/RunHopper.java` | Only referenced by `StaggerHopper` — transitively dead. Gated on D-3 |
| C-4 | `ShooterValuesSenable` | `subsystems/shooter/` | Zero call sites |
| C-5 | `getEstimationStdDevs()` | `PhotonVisionIO`, `PhotonVisionSimIO` | Zero callers in both. Deleting also orphans `VisionConstants.kSingleTagStdDevs` / `kMultiTagStdDevs` and their "measure in Stage 8" TODO — **decide whether to wire it up instead**, since `Vision.periodic()` currently computes its own inline std devs |
| C-6 | `gccPub` / `gcdPub` | `Vision.java` | Gated on camera names containing `"gcc"`/`"gcd"` — Limelight-era names. Current cameras: `Thrifty_cam_1`, `Thrifty_cam_2`, `photonvision`. Never fire |
| C-7 | `getTurretPose()`, `isFarEnough()`, `setCurrentZone()`, `switchState()`, `underTrench()` | `RobotStateMachine.java` | Zero callers. `underTrench()`'s Javadoc says "used by the flywheel" — `Flywheel.java` deleted Stage 5. **Note: `underTrench()` encodes real field geometry** — preserve the coordinates in a comment or plan doc before deleting |
| C-8 | `photonVisionIO` field | `RobotContainer.java` | Declared, never assigned — permanently null. The `REAL` branch uses locals |
| C-9 | 3 × `SlewRateLimiter` | `RobotContainer.java` | `filterXLimiter`/`filterYLimiter`/`filterRotLimiter` — declared, never used |
| C-10 | `kTiltPitch`, `NeoMotorConstants`, `AutoConstants.config`, `DriveConstants` SPARK MAX CAN IDs | `Constants.java` | Zero usages. **Do not delete `DriveConstants.kDriveKinematics`** until R1-A2 lands — it is currently load-bearing |
| C-11 | `SmartDashboard.putNumber("Shoot Speed", 0)` | `RobotContainer` constructor | Leftover from deleted `UpToSpeedHopperShoot` |
| C-12 | `kRobotToCam` | `Constants.VisionConstants` | Zero usages **and** contains a latent bug: `new Rotation3d(0, 0, 180)` — `Rotation3d` takes **radians**; 180 rad ≈ 28.6 revolutions. Delete it, or fix to `Math.PI` if a camera transform is wanted later. Leaving it as-is is the worst option |
| C-13 | `NamedCommands("SpeedUp")` = `Commands.none()` | `RobotContainer` | Any auto path referencing `SpeedUp` silently does nothing. Either implement or remove the registration **and** confirm no `.auto` file references it |

### Verification
Compile; `grep` each removed symbol to confirm zero remaining references; confirm no new
"unused" warnings appear in the IDE diagnostics.

---

# Stage R5 — Logging additions

**Risk: LOW.** Purely additive. This is the "nice to have" set specifically requested.
Do **R2 first** so new per-loop suppliers are not layered on an uncached pipeline.

Ordered by value:

## R5-1 — Command lifecycle logging  ★ highest value
`CommandScheduler.getInstance().onCommandInitialize(...)`, `.onCommandFinish(...)`,
`.onCommandInterrupt(...)` → write command name + event + timestamp to the DataLog.

**Why first:** makes autonomous debugging tractable. You would see exactly which
`NamedCommand` fired at which waypoint, what interrupted what, and whether a `withTimeout`
expired or the command finished on its own. Directly addresses the still-open
`SIM_PROGRESS.md` S-6 auto-validation work.

Watch: `onCommandInitialize` fires for every command including short-lived ones — string
logging every loop could be noisy. Log the name once per event, not per loop.

## R5-2 — `PowerDistribution`
`new PowerDistribution(...)` → total current, total energy, per-channel currents, PDH input
voltage and temperature. Register through `OnboardLogger`.

**Why:** you now log per-motor energy but have no ground-truth total to validate it against.
Per-channel current also identifies which breaker a fault sits behind.

## R5-3 — Match context (one-shot at DS connect)
`DriverStation.getEventName()`, `getMatchType()`, `getMatchNumber()`, `getReplayNumber()`,
`getAlliance()`, `isFMSAttached()`. Log once when FMS data becomes available.

**Why:** without it, a `.wpilog` cannot be tied to a specific match after an event.

## R5-4 — Loop timing
Log actual loop duration each cycle. You have fought overruns repeatedly; the only current
evidence is transient console warnings.

**Why:** turns overruns into a plottable trend in AdvantageScope. Note the trap found earlier
this session — a `<Subsystem>.periodic()` Tracer epoch also covers that subsystem's
`simulationPeriodic()`; label anything added here to avoid repeating that confusion.

## R5-5 — Robot health to `.wpilog`
`RobotController.isBrownedOut()`, battery voltage, and the full `CANStatus` — including
`busOffCount`, `txFullCount`, `receiveErrorCount`, `transmitErrorCount`, not just
`percentBusUtilization`.

**Why:** battery voltage currently goes only to SmartDashboard at 10 Hz, so it is not in the
durable log; and the four CAN error counters are the fields that actually diagnose a flaky
bus.

## R5-6 — Vision diagnostics
Tag count, best-target ambiguity, and a **rejected-measurement counter** in `Vision.periodic()`.

**Why:** the >4 m rejection filter currently discards silently. Without a counter you cannot
distinguish "vision is quiet" from "vision is being rejected constantly."

## R5-7 — Energy reset semantics + aggregate  (gated on **D-4**)
Add reset-on-enable (or per-match) to `OnboardLogger.registerEnergy`, and a summed
whole-robot energy/power total across all 16 motors.

**Why:** cumulative-since-boot totals are not comparable between matches; an aggregate gives
a per-match battery budget.

### Verification (all of R5)
Run sim, pull the `.wpilog`, open in AdvantageScope, confirm each new topic exists **and
carries changing values** — not merely that it appears. Re-check loop timing after the
additions to confirm no new overruns.

---

# Stage R6 — Tests

**Risk: NONE.** Purely additive.

`src/test/java/AlignTest.java` is `public class AlignTest {}` — an empty class. JUnit is
configured in `build.gradle` and `./gradlew test` passes vacuously, so the project appears
tested with zero coverage.

Highest-value targets, all pure functions needing no hardware:

| Target | Why |
|---|---|
| `Turret.findCC(position, reference, min, max)` | Multi-turn wrap-around with non-obvious edge cases. **Already made package-private static specifically so it could be unit-tested** — the seam was built and never used. Test: same-rotation, wrap positive, wrap negative, out-of-range clamp, exactly-0.5 boundary |
| `ToFAim.update()` | Convergence behavior. Test: stationary robot converges in 1 iteration; out-of-range distance returns `Impossible`; constraint violation returns `Impossible` |
| `LeadCompensator.computeLeadTarget()` | Test: zero velocity returns `hubPose` **exactly**; non-zero velocity shifts opposite the direction of travel; an `Impossible` inner strategy breaks the loop cleanly |
| `RobotStateMachine.checkZone()` | Pure geometry given a pose. Test each zone boundary and both alliances |

Delete or rename `AlignTest.java` — an empty file named after a deleted command
(`AlignTurretToHub`) is itself misleading.

---

## Commit strategy

One commit per stage, except R1 (one commit per finding — each needs independent
verification, and A1/A2 change pose behavior).

| Commit | Content |
|---|---|
| `review-r1-a1-field-layout` | Unify AprilTag layout |
| `review-r1-a2-kinematics` | Fix pose estimator kinematics |
| `review-r1-a3-telemetry` | Wire or delete `Telemetry` + correct `README.md` and the `configureLogging()` Javadoc |
| `review-r2-aim-cache` | Cache `getAimParams()`; de-allocate `tracked()` |
| `review-r3-side-effects` | Split `getState()`; de-duplicate `refreshPoseFromVision()` |
| `review-r4-dead-code` | Section C sweep |
| `review-r5-logging` | May split per sub-item if large |
| `review-r6-tests` | Unit tests + remove `AlignTest.java` |

---

## Out of scope

- Re-enabling CTRE `SignalLogger` (decided against in `logging_plan.md`).
- Migrating the deprecated `PhotonPoseEstimator` 3-arg constructor — deliberately kept, see
  `SIM_PROGRESS.md` S2-2.
- Stage 8 hardware work (CAN assignment, encoder zeroing, PID tuning, TOF re-measurement).
- Re-validating `ShooterConstants.scoringMeasurements` — carried over from the Hackbots
  robot and already tracked as Stage 8-8.
- Removing the REVLib vendordep — noted as legacy in `README.md`; removal is a separate
  decision with build implications.

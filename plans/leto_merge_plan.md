# Leto → leto_main Branch Integration Plan

**Status:** Planning only. No source files have been modified to produce this document — everything below comes from read-only `git log`/`git diff`/`git show` inspection of the `leto` and `leto_main` branches as they exist on 2026-09-16.

**Companion tracker:** [`LETO_MERGE_PROGRESS.md`](LETO_MERGE_PROGRESS.md) — update it in the same commit as any work this plan produces, per the process convention established in `leto`'s `plans/PLANNING_GUIDE.md` (see §0.3).

---

## 0. Ground rules

### 0.1 Scope and direction
The goal is to bring the **current tip of `leto`** into `leto_main`. Per explicit instruction, this plan disregards the fact that `leto` has not incorporated `leto_main`'s recent commits — `leto`'s current source tree is treated as the **target architecture**, and `leto_main`'s unique work is material to be **ported forward onto it**, not the other way around.

### 0.2 This is a planning document only
No code, config, or asset files are touched by this plan. Every file path, line count, and commit hash cited below was captured by direct inspection (`git diff --stat`, `git diff --name-only --diff-filter=...`, `git show <ref>:<path>`) and is reproducible by re-running the commands in Appendix A.

### 0.3 Process conventions to carry into execution
Once execution begins (a separate, explicitly-authorized phase), follow the same discipline this repo already uses on `leto` (see `leto`'s `plans/PLANNING_GUIDE.md`, which this very merge will bring onto `leto_main`):
- Update the progress tracker in the *same commit* as the code it describes.
- Cite concrete file/method names and commit hashes, never vague prose.
- When a later step invalidates an earlier claim in this plan, amend that claim in the same commit rather than leaving it stale.
- Compile (and run tests) after every reconciliation step; never mark a task done on "looks right."
- Verify vendor/WPILib/PhotonLib API signatures against extracted sources before relying on them, per this session's own hard-won lesson.

### 0.4 Reference points
| Ref | Meaning | Value |
|---|---|---|
| Merge base | Last common ancestor of `leto` and `leto_main` | `67ee7a5` |
| `leto` tip | Target architecture | `22ba3b1` — "Remove Telemetry.java's legacy Field2d array publishing" |
| `leto_main` tip | Current branch, holds competition-tested assets to port forward | `5372905` — "Merge pull request #13 from 6500gearcats/ShootOnTheMoveAuto2026" |
| Commits, base→`leto` | 78 | |
| Commits, base→`leto_main` | 88 | |
| Files changed, base→`leto` | 218 files, +36350/−4284 | |
| Files changed, base→`leto_main` | 66 files, +5346/−682 | |

---

## 1. Why this is not a normal `git merge`

`leto` and `leto_main` diverged from `67ee7a5` and then evolved in **fundamentally different directions on the same subsystems**:

- **`leto`** performed a full architectural rewrite of shooting/aiming/vision-estimation: it deleted the old `Flywheel`/`RangeFinder`/`AlignTurretToHub`/`ShootingSequence*` command family and the direct-TalonFX `Turret`, and replaced them with an IO-interface architecture (`aiming/` package — `AimStrategy`, `ToFAim`, `LeadCompensator`, `AimConstraints`, `AimParams`, `PhysicsAim`, `TuneAim`; `subsystems/shooter/` — `Shooter`, `ShooterIO`/`ShooterIOHardware`/`ShooterIOSim`; `subsystems/turret/TurretIO`/`TurretIOHardware`/`TurretIOSim`/`TurretIODisabled`; `superstructure/StateManager`). It also added a full review/remediation effort (`plans/`), a Javadoc site (`docs/`), a root `README.md`, and 4 new JUnit test files — none of which exist on `leto_main` at all.
- **`leto_main`** kept the *old* architecture alive and kept tuning it for real competition use: `ShootOnTheMoveAuto2026` was merged (PR #13) with real PhotonVision pose-ambiguity-rejection logic, new PathPlanner/Choreo autos, camera-transform fixes, and new commands (`HomeIntake`, `RunHopperBack`).

A textual `git merge leto` (in either direction) would hit conflicts on the ~20 files both branches touched, and — more dangerously — **silent semantic conflicts** on files that merge cleanly but shouldn't: both branches independently built PhotonVision pose-rejection logic under similar-but-not-identical implementations (see §3.4.1), and `leto_main` regenerated Tuner X drivetrain constants that overlap with CAN-ID work done only on `leto` (see §3.4.6).

**Recommended strategy — "leto-as-target, port-forward":** not a merge commit. Branch from `leto`'s tip, then deliberately re-apply `leto_main`'s still-relevant, field-tested behavior onto that tree, file by file, with an explicit accept/port/discard decision recorded for each one (mirroring the D-1..D-6 decision log style already used in `leto`'s `plans/review_plan.md`). Only once every file below has a recorded decision and the result compiles and passes tests does the branch replace/fast-forward `leto_main`.

---

## 2. File-by-file classification

Every file either branch touched relative to the merge base falls into one of four buckets. Each bucket gets its own reconciliation strategy in §5.

### 2.1 Bucket A — `leto`-only additions (accept as-is; `leto_main` has no equivalent)

No competing content exists on `leto_main`, so these are pure copies once the integration branch is cut from `leto`. Risk is low; the only real decision is §4.1 (generated docs).

| Category | Files |
|---|---|
| Aiming package | `aiming/AimConstraints.java`, `AimMeasurement.java`, `AimParams.java`, `AimStrategy.java`, `LeadCompensator.java`, `PhysicsAim.java`, `ToFAim.java`, `TuneAim.java` |
| Shooter package | `subsystems/shooter/Shooter.java`, `ShooterConstants.java`, `ShooterIO.java`, `ShooterIOHardware.java`, `ShooterIOSim.java` |
| Turret IO abstraction | `subsystems/turret/TurretIO.java`, `TurretIODisabled.java`, `TurretIOHardware.java`, `TurretIOSim.java`, `TurretConstants.java` |
| Superstructure | `superstructure/StateManager.java` |
| Utilities | `util/OnboardLogger.java`, `util/StatusSignalUtil.java` |
| New commands | `commands/AimPrep.java`, `commands/ShootWhenReady.java` |
| Vision/drivetrain misc | `subsystems/vision/LocalizationConstants.java`, `subsystems/drivetrain/SysIDUtil.java` |
| Docs & process | `README.md`, `docs/**` (~90 generated Javadoc HTML files), `plans/**` (23 files: `PLANNING_GUIDE.md`, `review_plan.md`/`REVIEW_PROGRESS.md`, `SIM_PROGRESS.md`, etc.) |
| Tests | `src/test/java/frc/robot/RobotStateMachineTest.java`, `src/test/java/frc/robot/aiming/LeadCompensatorTest.java`, `ToFAimTest.java`, `src/test/java/frc/robot/subsystems/turret/TurretTest.java` |

### 2.2 Bucket B — `leto_main`-only additions (must be ported onto `leto`'s tree)

Real, competition-relevant content with no home yet on `leto`.

| Category | Files | Note |
|---|---|---|
| New commands | `commands/HomeIntake.java`, `commands/RunHopperBack.java` | Depend on `Intake.deployIntake()`/`getDeployPos()`, `Hopper.startAllMotors()`/`stopAllMotors()`, `RobotStateMachine.checkZone()`/`isActive()`/`FieldZone` — **must verify these signatures still exist on `leto`'s rewritten `Intake`/`Hopper`/`RobotStateMachine` before porting** (§5.2). |
| Choreo trajectories | `AllianceToTrench.traj`, `NeutralToAlliance.traj`, `NeutralToTrench.traj`, `TrenchToNeutral.traj`, `TrenchToNeutral2ndPass.traj`, `Weirdmageddon.chor` | Data files, safe to copy; only matter if referenced by a surviving auto. |
| PathPlanner autos | `Armageddon.auto`, `CopiumRight.auto`, `Copy of Armageddon.auto`, `DeadNuts.auto`, `ShootOnTheMove.auto`, `SweepTest.auto`, `Weirdmagaddon.auto`, `WiggleTest.auto` | See §4.2 — most reference named commands `leto` deleted. |
| PathPlanner paths | ~25 files under `pathplanner/paths/` (`AM2AllianceToTrench.path`, `COP1_GrabFuel.path`, `DNNeutralToZone.path`, `WMTrenchToNeutral.path`, etc.) | Data files backing the autos above; port alongside their auto. |

### 2.3 Bucket C — Deleted on `leto`, still edited on `leto_main` (highest semantic risk)

`leto` removed these; `leto_main` kept using and tuning them. Each row is a real decision, not a mechanical merge.

| File | `leto_main` status | Decision required |
|---|---|---|
| `utility/RangeFinder.java` | Untouched since base (holds a 13-point hand-tuned distance→shot-angle `InterpolatingDoubleTreeMap`) | **Port the 13 calibration points** (2.23→50°, 1.8→45°, 2.0→48°, 2.5→55°, 2.8→57°, 3.0→58°, 3.2→61°, 3.4→63°, 3.5→69°, 3.6→73°, 4.0→75°, 4.2→81°, 5.2→100°) into `leto`'s replacement calibration table (`ShooterConstants.scoringMeasurements`, consumed by `ToFAim`) — this is tuned field data, not code, and has no automatic equivalent. |
| `subsystems/turret/Flywheel.java` | Still edited, +72/−0 lines since base | Confirm every tuned behavior (shot multiplier, rotation multiplier, "under trench" detection, `reqSpeed`/`actSpeed` split) has a real equivalent in `Shooter`/`ShooterConstants`; port any gain/multiplier that doesn't. |
| `commands/AlignTurretToHub.java` | Still edited, +30/−? lines since base | `leto` replaced this whole family with `AimPrep`/`ShootWhenReady` + `Shooter.isTracked()`/`Turret.isTracked()`. Diff `leto_main`'s recent edits specifically (not just the base diff) to find anything not yet represented in the new commands, including the `ductTapeCorrection` offset behavior. |
| `commands/ShootingSequence.java`, `ShootingSequenceUTS.java` | Actively used — backs 13 of `leto_main`'s `NamedCommands` (`ShootFuel*`, `NewShootFuel*`, `ManualShootFuel*`) | Confirm `AimPrep`/`ShootWhenReady` cover the full shoot sequence (hopper feed timing, flywheel spin-up gating, shot-ready check) before treating these as fully superseded. |
| `commands/UpToSpeedHopperShoot.java`, `BurstFire.java`, `MoveTurret.java`, `SetTurretAngle.java`, `ShootFuel.java` | `BurstFire.java` deleted on both sides (safe); the rest still exist on `leto_main` | Same family as above; confirm no unique behavior is stranded. |
| `subsystems/Climber.java`, `commands/ClimbPole.java` | Still referenced live — `leto_main`'s `RobotContainer.java` registers `"Climb"`, `"ClimbUp2s"`, `"ClimbDown2s"` NamedCommands against `m_climber` | Consistent with this session's very first task on `leto` ("no climber exists on this robot, remove `ClimbUp2s`/`ClimbDown2s`"). **Re-confirm that decision still holds** — if `leto_main`'s hardware genuinely has no climber, drop `Climber.java`/`ClimbPole.java` and the 3 NamedCommands; if a climber was added back, this is a hard blocker requiring new code, not a port. |
| `subsystems/LedCANdle.java`, `commands/CoolSnurbo.java`, `UncoolSnurbo.java` | Untouched since base | Confirm LEDs are still installed/wired before deciding to drop vs. port. |
| `subsystems/vision/limelight/LimelightIO.java`, `LimelightHelpers.java` | `leto_main`'s `Vision.java` still imports `subsystems.vision.limelight.LimelightIO` and calls `throttleLimelight()`/`resetLimelightThrottle()` | `leto` dropped Limelight support entirely (PhotonVision-only). Confirm no Limelight hardware remains; if it does, Limelight support must be re-added to the new `VisionIO` architecture, not silently dropped. |
| `utility/ShooterValuesSenable.java`, old `subsystems/SysIDUtil.java` | Untouched since base, no `leto_main` edits | Low risk — grep the full `leto_main` tree for any remaining reference before discarding (`leto` already re-added a *different* `SysIDUtil.java` under `subsystems/drivetrain/`). |
| `src/test/AlignTest.java` | Empty class, already deleted on `leto` this session | No action needed — confirmed dead on both lineages. |

### 2.4 Bucket D — Modified on both sides (real reconciliation, not deletion)

| File | `leto` diff vs base | `leto_main` diff vs base | Reconciliation approach |
|---|---|---|---|
| `RobotStateMachine.java` | 539 lines | 184 lines | **Highest risk, highest centrality — handle last.** See §3.4.2. |
| `RobotContainer.java` | 516 lines | 211 lines | Binding-by-binding merge; every `leto_main` controller binding and `NamedCommands` entry needs an explicit carry-forward/drop/replace decision. See §3.4.3 and §4.2. |
| `Turret.java` | 457 lines | 38 lines | `leto_main`'s version (read in full this session) is the direct-TalonFX implementation: 3 PID slot configs, live SmartDashboard-driven gain tuning (`slot1Configs.kV = SmartDashboard.getNumber("kV", 0)`, etc.), limit-switch zeroing (`toZeroPos`/`m_switch`/`zeroMotorPosition()` with its documented `-1` offset correction), and `ductTapeCorrection`. See §3.4.4 — every one of these needs a confirmed equivalent in `leto`'s `TurretIOHardware`. |
| `subsystems/vision/photonvision/PhotonVisionIO.java` | 79 lines | 461 lines | **Both sides independently built near-identical pose-rejection logic.** See §3.4.1 — do not let either version simply "win"; reconcile heuristics. |
| `Vision.java` | 271 lines | 15 lines | `leto` added an `SwerveDriveKinematics` constructor param and deleted `gccPub`/`gcdPub` NT publishers; `leto_main` still populates `gccPub`/`gcdPub` every loop. Confirm nothing (dashboard layout, AdvantageScope config) depends on the `StateMachine/GCC`/`StateMachine/GCD` topic names before accepting the deletion. |
| `subsystems/vision/photonvision/PhotonVisionSimIO.java` | 103 lines | 2 lines | Low risk — `leto_main`'s diff is trivial. **Do not migrate the deprecated 3-arg `PhotonPoseEstimator` constructor** — standing decision from this session, recorded as `SIM_PROGRESS.md` S2-2 on `leto`. |
| `subsystems/vision/VisionIO.java` | 28 lines | 6 lines | `leto` added `getTagCount()`/`getBestTargetAmbiguity()` to the interface. Confirm `leto_main`'s 6-line addition (likely also interface growth) doesn't collide with or duplicate these. |
| `subsystems/vision/VisionEstimate.java` | 34 lines | 17 lines | Diff both directly; small file, moderate risk only because it's a shared value type consumed by the reconciled `PhotonVisionIO.java`. |
| `Constants.java` | 255 lines (net *shrink* — deleted `DriveConstants`/`ModuleConstants`/`OIConstants`/`AutoConstants`/`NeoMotorConstants`/`GyroConstants`) | 15 lines (additive, against the *old*, unshrunk shape) | Grep exactly what `leto_main`'s 15-line diff added; confirm each new constant has a home in `leto`'s trimmed `Constants.java` before accepting the deletions wholesale. |
| `subsystems/hopper/Hopper.java` | 89 lines | 4 lines | `leto_main`'s tiny diff is likely a targeted bugfix/tuning tweak — diff it in isolation and confirm it survives in `leto`'s larger rewrite. |
| `subsystems/intake/Intake.java` | 80 lines | 44 lines | Same approach; `leto_main`'s diff is large enough here to need real side-by-side reading, not just a spot check. |
| `commands/RunHopper.java` | 52 lines | 2 lines | **Concrete, already-identified conflict:** `leto_main` tuned `m_hopper.startAllMotors(-0.9, 1)` → `startAllMotors(-1, 1)` (full-speed reverse feed). Confirm `leto`'s rewritten `RunHopper.java` uses `-1`, not the older `-0.9`, or the tuning regresses silently. |
| `commands/RunIntake.java` | 30 lines | 16 lines | **Concrete, already-identified conflict:** `leto_main` added a second constructor overload `RunIntake(Intake, double speed, double deploySpeed)` (deploy speed defaults to `0.15`, but callers like `HomeIntake`/`BopBop` need `0.25`/`0.35`/`-0.15`). Confirm `leto`'s version preserves this overload — `HomeIntake` from Bucket B depends on `Intake.deployIntake()` directly, but other `leto_main` call sites use the `RunIntake` 3-arg constructor and would silently lose the custom deploy speed if only the 2-arg constructor survives. |
| `generated/TunerConstants.java` | 2 lines | 169 lines | **`leto_main`'s version is very likely the one to keep or refresh.** A 169-line diff from a 2-line one strongly suggests a real Tuner X re-export (fresh CAN IDs / PID gains / wheel radius) against the *current physical robot*, while `leto`'s tiny diff is unrelated. See §3.4.6. |
| `generated/TunerConstants2.java` | 2 lines | **file deleted** (commit `9cf13e5`, "fixed snurbo and removed artemis constants") | Hard blocker — needs a human hardware answer. See §3.4.6 and Open Question Q1. |

---

## 3. Deep-dive on the highest-risk items

### 3.4.1 PhotonVisionIO.java — parallel evolution of the same feature

`leto`'s current `PhotonVisionIO.java` (read in full this session) already contains:
- `MAX_SINGLE_TAG_AMBIGUITY` / `MAX_SINGLE_TAG_DISTANCE_METERS` / `MAX_POSE_Z_METERS` rejection thresholds in `shouldRejectEstimate(EstimatedRobotPose)`.
- Coprocessor-multi-tag-first estimation (`estimator.estimateCoprocMultiTagPose(result)`, falling back to `estimateLowestAmbiguityPose(result)`).
- A `CamPoseRejected<camera>` SmartDashboard boolean.
- A full CSV field-calibration data-collection tool (`VisionTrialLogger`, dashboard-driven start/stop/label/notes, writes to `vision-data/photonmq-<trialId>.csv`).

`leto_main`'s recent commit history (`5a915c1` "accepts single tag estimates", `519c8b0` "Rejecting ambiguous posees", `50e7ff8` "CamPoseRejected logging") shows it **independently built the same category of feature** — same `CamPoseRejected` dashboard-key naming convention, same general ambiguity-rejection concept — but as a textually distinct 461-line diff.

**This will not show up as a git conflict if merged carelessely, because it's a whole-file rewrite on both sides — whichever version "wins" silently discards the other's tuning.** Before reconciling:
1. Read `leto_main`'s full `PhotonVisionIO.java` (only method signatures were sampled this session) side-by-side with `leto`'s.
2. Identify any rejection heuristic, threshold value, or field-tuned constant present in `leto_main`'s version that `leto`'s lacks (e.g., different ambiguity cutoff learned from real matches).
3. Fold in anything `leto_main` learned; keep `leto`'s cleaner IO-interface shape and its `VisionTrialLogger` (no evidence `leto_main` has an equivalent).

### 3.4.2 RobotStateMachine.java — handle last

Both branches heavily modify this singleton god-object (539 lines on `leto`, 184 on `leto_main`). `leto`'s version was itself substantially reworked earlier this session (static-field-init-order crash fix, `computeAimParams()`/`getAimParams()` caching, several dead getters deleted, `Vision`/`Flywheel`/`Turret` wiring changed). It has a **known, unresolved bug** carried over from that work: `checkZone()`'s Y-boundary logic (`y > 4.2` → `NEUTRAL_BOTTOM`, `y < 3.8` → `NEUTRAL_TOP`) is inverted relative to the `FieldZone` enum's own Javadoc — tracked as decision D-6 in `leto`'s `REVIEW_PROGRESS.md`, unresolved, needs a human with the field diagram.

`leto_main`'s `RobotContainer.java` and multiple commands (`RunHopper`, `RunHopperBack`) call `stateMachine.checkZone()` and compare against `FieldZone.ALLIANCE` directly in gating logic (`if ((!stateMachine.isActive()) && (stateMachine.checkZone() == FieldZone.ALLIANCE)) { return; }`). **Do not reconcile `RobotStateMachine.java` before D-6 is resolved** — porting `leto_main`'s zone-gated commands onto a still-buggy `checkZone()` will silently reproduce the same bug in every command that depends on it.

Recommendation: reconcile this file only after every file it depends on (`Turret`, `Vision`, `Shooter`/`Flywheel`) is already settled, and only after D-6 has an answer.

### 3.4.3 RobotContainer.java — 25 NamedCommands to reconcile

`leto_main`'s `RobotContainer.java` constructor registers 25 `NamedCommands`, at least 13 of which construct instances of classes `leto` deleted (`ShootingSequence`, `ShootingSequenceUTS`, `AlignTurretToHub`, `ClimbPole`, `SetTurretAngle`):

```
AlignTurretFromRightTrench, TurretDeadOn, AlignTurretFromLeftTrench, IntakeFuel,
DeployIntakeFast, DeployIntakeFast0.5s, IntakeFuelJason, Intake, IntakeLong,
HomeIntake, ShootFuel, ShootFuel3s, ShootFuel10s, ShootFuel7s, ShootFuel5s,
NewShootFuel3s, ManualShootFuel3s, ManualShootFuel, TrenchStartAngle,
NewShootFuel5s, NewShootFuel10s, NewShootFuel4s, NewShootFuel8s, AlignTurret,
AlignTurret1s, Climb, BopBop, BopBopStayUp, SpeedUp, ClimbUp2s, ClimbDown2s
```

Each of these needs one of three outcomes on the integration branch:
- **Keep as-is** — the backing class survives unchanged on `leto` (e.g. `IntakeFuel`/`Intake`/`IntakeLong` back onto `RunIntake`, which `leto` still has, modulo §2.4's overload concern).
- **Re-point** — rewrite the registration to call `leto`'s replacement (e.g. `ShootFuel*`/`NewShootFuel*` → some composition of `AimPrep`/`ShootWhenReady`; `AlignTurret*`/`TrenchStartAngle`/`AlignTurretFrom*Trench` → `AimPrep` or a direct `Turret`/`Shooter` call).
- **Retire** — the command was climber-specific (`Climb`, `ClimbUp2s`, `ClimbDown2s`) and should be dropped per the no-climber decision already made once this session (pending re-confirmation, §2.3).

This reconciliation must happen *before* §5.5 (auto/path audit), since every `.auto` file that names one of these strings will silently no-op (or throw at runtime) if the name is retired without a replacement being registered under the same name, or explicitly rewritten to use the new name.

### 3.4.4 Turret.java — hardware-calibration behavior most at risk of silent loss

`leto_main`'s `Turret.java` (read in full this session) is the pre-rewrite, direct-`TalonFX` implementation. It contains real, bench-tuned behavior that has no obviously-visible equivalent unless specifically checked in `leto`'s `TurretIOHardware.java`:

1. **Three PID slot configurations** — Slot0 (kS=0.2, kV=5, kA=3, kP=3, kD=0.4), Slot1 (live-tunable via `SmartDashboard.getNumber("kV"/"kA"/"kP"/"kD", 0)` — an active tuning workflow, not just constants), Slot2 (kS=0.20757, kV=0.1034, kA=0.0075573, kP=7.4749, kD=0.36566, commented "This one is good" — i.e., the actual competition-ready gains).
2. **Limit-switch zeroing routine** — `goToZero()`/`toZeroPos` drives the motor at `-0.5` until `DigitalInput` channel 4 trips, then calls `zeroMotorPosition()`, which sets position to `-1` rather than `0` with an inline comment explaining the limit switch is "slightly inaccurate after zeroing the first time."
3. **Position conversion formula** — `getConvertedTurretPosition() = -((motorRotations * 4) - 110)` and its inverse `unconvertPosition(deg) = ((-1 * pos) + 110) / 4` — a specific gear-ratio/offset mapping.
4. **Soft-limit clamping in `setSpeed()`** — blocks further motion past raw position `2`/`53` unless `overridden` is toggled.
5. **`ductTapeCorrection`** — a `-5` degree offset applied in `setPosition()` when `robotStateMachine.ductTapeCorrection` is set.

Every one of these must be traced to a specific method in `leto`'s `TurretIOHardware.java`/`Turret.java` (or ported in if missing) before the integration branch is trusted on real hardware — this is exactly the category of change (raw calibration numbers, not logic shape) that a clean-room IO-abstraction rewrite is most likely to have dropped or reset to placeholder values.

### 3.4.5 Named-command / auto-file cross-reference (Bucket B + §3.4.3 combined)

A direct search of `leto_main`'s `src/main/deploy/pathplanner/autos/*.auto` files for the doomed command names found **17 auto files** with at least one reference: `Armageddon.auto`, `Artemis2.auto`, `CopiumRight.auto`, `DeadNuts.auto`, `FinalHailMary.auto`, `FinalHope.auto`, `HumanPlayer.auto`, `ModifiedWakeRight.auto`, `ProjectHailMaryRight.auto`, `ShootOnTheMove.auto`, `SimonAutoTest.auto`, `SweepTest.auto`, `WakeCenter.auto`, `WakeLeft.auto`, `WakeRight.auto`, `Weirdmagaddon.auto`, plus one under `autos/simple/center.auto`. Named commands are resolved by string at runtime via the `NamedCommands` registry, so a stale reference does not fail to compile — it silently no-ops or throws during a match. Every one of these needs re-validation once §3.4.3's re-pointing decisions are made (§5.5).

### 3.4.6 CAN ID / hardware constants — TunerConstants reconciliation

Earlier this session (on `leto`), a CAN ID collision was discovered and fixed: `ShooterConstants.kMotor1Id` (originally `30`) collided with `TunerConstants2`'s Pigeon2 IMU (also ID `30`); the fix moved the shooter motor to ID `36`. That fix's validity depends entirely on `TunerConstants2.java` still describing real hardware.

`leto_main` deleted `TunerConstants2.java` outright (commit `9cf13e5`, "fixed snurbo and removed artemis constants") and separately regenerated `TunerConstants.java` with a 169-line diff — almost certainly a fresh Tuner X export reflecting the robot's *current* CAN bus layout, including possibly-changed CAN IDs, PID gains, and wheel geometry.

Before Stage 4 (§5.4) can close, this needs:
1. A human answer to Open Question Q1 (§6) — does the second Pigeon2/CANivore drivetrain config still exist physically?
2. If no: drop `TunerConstants2.java` (matching `leto_main`), and **re-run the CAN ID collision check from scratch** against `leto_main`'s regenerated `TunerConstants.java` — the ID-30 collision that motivated moving the shooter motor to ID 36 may no longer exist (good), or a *new* collision may have been introduced by the regeneration (bad) — this cannot be assumed either way without checking.
3. If yes: `leto_main`'s deletion was itself a bug; keep `TunerConstants2.java` and re-run the same collision check.

---

## 4. Non-code assets requiring a decision

### 4.1 Generated Javadoc site (`docs/`, ~90 files, `leto`-only)
Decide: commit the generated HTML to source control as `leto` currently does, or move to a `.gitignore`d output directory regenerated by the existing Gradle `javadoc` task (`ca61167`/`e08c9af` on `leto`) on demand / in CI. Low risk either way; purely a repo-hygiene call, not a functional one.

### 4.2 New autos/paths whose named commands are being retired or re-pointed
Every file in Bucket B's auto/path list, plus the 17 files identified in §3.4.5, needs to be opened and checked for exactly which named-command strings it references, cross-referenced against the final registry produced by §3.4.3's reconciliation. Any auto file that cannot be made to work with the reconciled command set should be flagged for the user rather than silently dropped or silently left broken.

---

## 5. Stage-by-stage execution plan

*(This section describes the work to be done in a later, separately-authorized execution phase. No stage below has been started.)*

- **Stage 0 — Environment setup.** Create an integration branch (e.g. `leto-integration`) from `leto`'s tip (`22ba3b1`) — not from `leto_main` — since `leto` is the target architecture. Confirm a clean Gradle build on that branch alone before touching anything else.
- **Stage 1 — Land Bucket B as inert copies.** Copy `HomeIntake.java`, `RunHopperBack.java`, and all new Choreo/PathPlanner assets onto the integration branch. Compile-check only (`HomeIntake`/`RunHopperBack` will not compile until their dependencies are confirmed in Stage 2) — do not wire them into `RobotContainer.java` yet.
- **Stage 2 — Resolve Bucket C, one file at a time.** For each row in §2.3, record and apply the decision as its own commit, producing a decision log in the same style as `leto`'s `review_plan.md` (D-1..D-6). Blocked items (Climber, LEDs, Limelight — §6 Q1-Q3) get an explicit "blocked, pending human answer" entry rather than a guess.
- **Stage 3 — Reconcile Bucket D, easiest first.** Order: `VisionEstimate.java` → `VisionIO.java` → `PhotonVisionSimIO.java` → `Hopper.java`/`Intake.java` → `RunHopper.java`/`RunIntake.java` → `Constants.java` → `PhotonVisionIO.java` (§3.4.1) → `Turret.java` (§3.4.4) → `Vision.java` → `RobotContainer.java` (§3.4.3) → `RobotStateMachine.java` (§3.4.2, last, and only after D-6 is resolved).
- **Stage 4 — CAN ID / hardware-mapping audit.** Resolve §3.4.6 in full: settle `TunerConstants2.java`'s fate, then re-run the CAN collision check against whichever `TunerConstants.java` survives, plus the final `ShooterConstants`/`TurretConstants` CAN IDs.
- **Stage 5 — Auto/path audit.** For every file identified in §3.4.5 and §4.2, verify every named-command reference resolves against the Stage 2/3 registry; fix or retire any that don't.
- **Stage 6 — Build & test verification.** Full Gradle compile; full unit test suite (`leto`'s 17 existing tests: `RobotStateMachineTest` ×5, `LeadCompensatorTest` ×3, `ToFAimTest` ×3, `TurretTest` ×6, plus any surviving `leto_main` tests — currently only the already-dead `AlignTest.java`, which both branches agree is empty); a full simulation smoke test mirroring what was already exercised this session (sim GUI launch, joystick input, AdvantageScope pose visibility, Tracer loop-timing check).
- **Stage 7 — Real-robot bring-up checklist.** Bench-test every item flagged in Stage 2/3 as "hardware behavior, verify before trusting": turret zeroing routine, the three PID slots (especially Slot2's "This one is good" gains), `ductTapeCorrection`, the intake/hopper motor-direction tuning from §2.4's `RunHopper`/`RunIntake` conflicts, and the RangeFinder calibration-table port.
- **Stage 8 — Finalize.** Merge or fast-forward the integration branch into `leto_main` — this is itself a decision for the user (§6 Q4), since it rewrites a shared branch others may have checked out.

---

## 6. Open questions — decisions only a human can make

| # | Question | Blocks |
|---|---|---|
| Q1 | Does the physical robot still have the second Pigeon2/CANivore drivetrain that `TunerConstants2.java` describes, or was it removed (matching `leto_main`'s deletion in `9cf13e5`)? | Stage 4 (§3.4.6) |
| Q2 | Is a Limelight camera still mounted, or is the robot PhotonVision-only now (matching `leto`'s removal of `LimelightIO`/`LimelightHelpers`)? | Stage 2 (`LimelightIO.java` row) |
| Q3 | Is there a climber on the robot? (This session already decided "no" once, on `leto` only — `leto_main` still has `Climber.java`/`ClimbPole.java` wired live via 3 NamedCommands.) | Stage 2 (`Climber.java` row), Stage 3 (`RobotContainer.java`/`RobotStateMachine.java`) |
| Q4 | Are the CANdle LEDs (`LedCANdle.java`, `CoolSnurbo`/`UncoolSnurbo`) still installed and wired? | Stage 2 |
| Q5 | Preferred integration outcome: fast-forward `leto_main` to the reconciled branch, or an explicit merge commit that preserves both histories? | Stage 8 |
| Q6 | Commit the generated `docs/` Javadoc site to source control (as `leto` does today), or regenerate on demand and `.gitignore` it? | §4.1, low priority |

---

## Appendix A — Commands used to produce this plan (for reproducibility)

```
git merge-base leto leto_main
git log --oneline leto_main..leto
git log --oneline leto..leto_main
git rev-list --count 67ee7a5e..leto
git rev-list --count 67ee7a5e..leto_main
git diff --stat 67ee7a5e.. leto / leto_main   (overall, and restricted to the overlapping-file list)
git diff --diff-filter=D --name-only 67ee7a5e.. leto / leto_main
git diff --diff-filter=A --name-only 67ee7a5e.. leto / leto_main
git show leto:<path>   /   git show leto_main:<path>   (full-file reads of PhotonVisionIO.java, PhotonVisionSimIO.java,
    VisionIO.java, Vision.java, Turret.java, HomeIntake.java, RunHopperBack.java, RobotContainer.java excerpt)
git diff 67ee7a5e.. leto_main -- RunHopper.java / RunIntake.java
git log --oneline --diff-filter=D -- generated/TunerConstants2.java   (on leto_main)
grep -l "<named-command strings>" src/main/deploy/pathplanner/autos/*.auto   (on leto_main)
```

All raw output is reproducible; nothing above was inferred without a corresponding command.

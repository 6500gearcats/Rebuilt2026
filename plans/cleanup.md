# Rebuilt2026 — Post-Refactor Cleanup Plan

Identified during the Stage 6–7 + commenting pass survey. None of these items affect robot function today; they are dead code, stale state, and unused declarations that should be removed before Stage 8 hardware bringup to keep the codebase clean.

---

## Stage C-1 — Dead State in RobotStateMachine

### Problem

`updateTargetPose()` was deleted in Stage 5 during the Hackbots state machine wire-up. As a result:

- `targetPose` is initialized to `new Pose2d()` and is **never updated** — it always holds field-origin zeros.
- `targetPosePublisher` publishes those zeros to NT4 every 10 Hz tick, making AdvantageScope show a bogus "target" at (0, 0).
- `getTargetPose()` returns stale zeros to any caller.
- `isFacingHub()` reads `targetPose` and compares it against robot pose — the result is meaningless.

### Recommendation

**Option A (preferred):** Wire `targetPose` to `getHubPose()` — the field position of the scoring hub derived from `Tag_POSE2D`. If `Tag_POSE2D` is non-null, `targetPose = new Pose2d(Tag_POSE2D.getX(), Tag_POSE2D.getY(), new Rotation2d())`. This makes `isFacingHub()` meaningful again.

**Option B:** Remove `targetPose`, `targetPosePublisher`, `getTargetPose()`, and `isFacingHub()` entirely if nothing in the codebase calls them after cleanup.

### Files

- `src/main/java/frc/robot/RobotStateMachine.java`

### Tasks

| ID | Task | Done |
|----|------|------|
| C1-1 | Search all call sites of `isFacingHub()` and `getTargetPose()` | ⬜ |
| C1-2 | If no external callers: delete `targetPose`, `targetPosePublisher`, `getTargetPose()`, `isFacingHub()` | ⬜ |
| C1-3 | If callers exist: wire `targetPose` to `getHubPose()` / `Tag_POSE2D` in `periodic()` | ⬜ |
| C1-4 | Compile-verify and commit | ⬜ |

---

## Stage C-2 — Dead Private Method in RobotStateMachine

### Problem

`getBestPoseTarget()` is a private method in `RobotStateMachine`. It was never called from outside the class, and after the Stage 5 refactor it has no internal callers either. It is dead code.

### Recommendation

Delete the method. If the logic inside is relevant to a future feature (multi-tag arbitration, for example), move the algorithm to a comment in the plan rather than keeping dead code in the file.

### Files

- `src/main/java/frc/robot/RobotStateMachine.java`

### Tasks

| ID | Task | Done |
|----|------|------|
| C2-1 | Confirm `getBestPoseTarget()` has zero callers (grep) | ⬜ |
| C2-2 | Delete the method | ⬜ |
| C2-3 | Compile-verify and commit with C-1 changes | ⬜ |

---

## Stage C-3 — Unused Swerve Requests in RobotContainer

### Problem

Two `SwerveRequest` objects are declared as fields but never bound to any trigger or button:

```java
private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
```

These were likely placeholder bindings from the CTRE swerve template that were never implemented. The `Idle` request (used in disable) is already declared locally inside `configureBindings()`.

### Recommendation

**Option A (preferred):** Remove both fields. Neither `brake` nor `point` appears anywhere in the binding logic.

**Option B:** If brake-on-button-hold is a desired feature (e.g., driver B button → lock wheels while pushed), wire it: `joystick.b().whileTrue(drivetrain.applyRequest(() -> brake))`.

### Files

- `src/main/java/frc/robot/RobotContainer.java`

### Tasks

| ID | Task | Done |
|----|------|------|
| C3-1 | Grep for `brake` and `point` across the repo to confirm zero use | ⬜ |
| C3-2 | Remove both field declarations (or bind them if the feature is wanted) | ⬜ |
| C3-3 | Compile-verify and commit | ⬜ |

---

## Stage C-4 — Unused LedCANdle Instance

### Problem

```java
private LedCANdle m_candle = new LedCANdle();
```

`m_candle` is declared in `RobotContainer` but never passed to any subsystem, command, or binding. LED control lives entirely inside `RobotStateMachine` via color fields and a separate LED state machine — `RobotContainer` is not involved in that pipeline.

### Recommendation

**Option A (preferred):** Remove `m_candle` from `RobotContainer`. The `LedCANdle` instance in `RobotStateMachine` (if any) is the live one.

**Option B:** If LED control should be driven from `RobotContainer` (e.g., driver button changes LED color), wire it here and remove the duplicate in `RobotStateMachine`.

Before removing, confirm which class owns the live `LedCANdle` CAN connection — two instances on the same CAN ID will fight each other.

### Files

- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/RobotStateMachine.java` (check for duplicate instance)

### Tasks

| ID | Task | Done |
|----|------|------|
| C4-1 | Find all `LedCANdle` instantiations — confirm which one is the live device | ⬜ |
| C4-2 | Remove the dead instance (likely `RobotContainer.m_candle`) | ⬜ |
| C4-3 | Remove the now-unused import if any | ⬜ |
| C4-4 | Compile-verify and commit | ⬜ |

---

## Stage C-5 — SysID Stubs (Defer to Stage 8)

### Problem

```java
SysIDUtil m_turretSysID = new SysIDUtil();
SysIDUtil m_flywheelSysID = new SysIDUtil();
```

Both fields are instantiated in `RobotContainer` but `SysIDUtil` is a stub — no methods are called on either object. They were kept as placeholders for Stage 8 SysID characterization (turret and flywheel PID tuning on hardware).

### Recommendation

**Keep but don't act until Stage 8.** SysID characterization requires the assembled robot. When Stage 8 begins:

1. Implement `SysIDUtil` for turret: bind `joystick.a().whileTrue(m_turretSysID.sysIdQuasistatic(SysIdRoutine.Direction.kForward))` etc.
2. Do the same for flywheel.
3. Run SysID routines, export data, fit `kS`/`kV`/`kA`, update `TurretConstants` and `ShooterConstants`.

### Files

- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/utility/SysIDUtil.java`

### Tasks

| ID | Task | Done |
|----|------|------|
| C5-1 | (Stage 8) Implement `SysIDUtil.sysIdQuasistatic` / `sysIdDynamic` for turret | ⬜ |
| C5-2 | (Stage 8) Bind turret SysID routines to driver buttons (temp, for characterization session) | ⬜ |
| C5-3 | (Stage 8) Repeat for flywheel | ⬜ |
| C5-4 | (Stage 8) Run, export, fit constants, update `TurretConstants` / `ShooterConstants` | ⬜ |

---

## Suggested Order of Execution

Do C-1 through C-4 together in a single commit — they are all safe deletions/removals with no behavior change. Defer C-5 to Stage 8.

```
C-1 + C-2  →  RobotStateMachine dead state/method cleanup
C-3 + C-4  →  RobotContainer unused field cleanup
compile-verify both together → commit → push
C-5        →  Stage 8 (hardware available)
```

---

## Verification Checklist

Before committing each stage:

- [ ] `.\gradlew.bat compileJava` with WPILib JDK (`$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"`) — zero errors
- [ ] No yellow "unused variable" warnings for removed fields
- [ ] AdvantageScope NT4 table no longer shows bogus `StateMachine/TargetPose` at (0, 0) (C-1)
- [ ] No duplicate `LedCANdle` CAN ID warnings on robot console (C-4)

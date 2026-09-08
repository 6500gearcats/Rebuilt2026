# Rebuilt2026 — Documentation Plan

Goal: bring every class and public method to tutorial-level Javadoc so a new team member
can read any file and understand what it does, why it exists, and what non-obvious
constraints apply.

Track execution in `DOC_PROGRESS.md`.

**Branch:** `leto`  
**Standard:** every public class gets a class-level Javadoc; every public/package-visible
method gets a Javadoc with `@param`, `@return`, and a WHY clause when the behavior is
non-obvious. Private methods are documented when the logic would surprise a reader.

---

## What Already Meets the Bar (skip these)

| File | Notes |
|------|-------|
| `Turret.java` | Best in codebase — tutorial quality throughout |
| `ToFAim.java` | Excellent physics rationale |
| `LeadCompensator.java` | Excellent |
| `Shooter.java` | Excellent |
| `RobotContainer.java` | Excellent |
| `StateManager.java` | Excellent |
| `AimParams.java` | Good |
| `ShootWhenReady.java` | Good |
| `AimPrep.java` | Good |
| `TurretIO.java` | Good |
| `subsystems/drivetrain/Telemetry.java` | Class + constructor + method Javadoc in place |
| `subsystems/vision/LocalizationConstants.java` | Class + field Javadoc adequate for a placeholder file |
| `subsystems/turret/TurretConstants.java` | Class Javadoc + field docs; Stage 8 TODOs clearly marked |
| `aiming/AimConstraints.java` | Simple record; existing docs sufficient |
| `generated/TunerConstants*.java` | Auto-generated — never hand-document |

---

## D-1 — Priority 1: Complex Logic, Actively Misleading or Zero Docs

These carry the highest risk: a reader modifying them without documentation will likely
break behavior.

### D1-1 `PhysicsAim.java`

**Why critical:** Contains the full 3D ballistic kinematics solver. `quicksolve()` derives
flight time from a quadratic equation, then back-calculates `vx/vy/vz`. `update()` wraps
this in a 5-iteration binary search over arc height, with a yaw sanity-check that silently
discards shots outside `0.8 * PI`. Zero documentation anywhere in the file.

**Tasks:**
- Class Javadoc: explain physics model, how it differs from `ToFAim`, when you use it
- Constructor: document `minDescentVelocity` / `maxDescentVelocity` and physical meaning
- `quicksolve()`: explain the quadratic derivation (`a = 0.5 * 9.81`, `b = -finalDescentSpeed`, `c = -dz`), units for all params and return
- `update()`: explain the binary-search loop, the yaw sanity-check threshold, and the `impossible()` exit conditions

**File:** `src/main/java/frc/robot/aiming/PhysicsAim.java`

---

### D1-2 `RobotStateMachine.java` — `getState()` mismatch + private methods

**Why critical:** `getState()` is documented as "Returns the current robot state" but is a
250-line method that also calls `setState()` as a side effect and drives the LED state
machine. Actively misleading.

**Tasks:**
- `getState()`: rewrite Javadoc to accurately describe all side effects (LED update, state transitions)
- `newPostedValue()`: document the LED blink state machine — what red/green/white flashes mean
- `checkAlliance()`: explain AprilTag 10 vs 20 selection, the 0.5842 m hub offset, and why hub center ≠ tag center
- `isFacingHub()`: explain `Math.PI` subtraction (flips from robot-forward to shooter-facing direction)
- `underTrench()`: add field diagram reference and explain what the four coordinate rectangles represent
- `isFarEnough()`: document the 4.2 m threshold and what "far enough" gates
- Minor stubs: `getHubPose()`, `getTurretPose()`, `getFieldSpeeds()`, `getChassisSpeeds()`, `distToTag()`, `switchState()`, `getGameData()/setGameData()/hasData()`, `getAlliance()`, `isActive()`, `isInAlliance()`
- Enum values: `RobotState`, `FieldZone`

**File:** `src/main/java/frc/robot/RobotStateMachine.java`

---

### D1-3 `Robot.java`

**Why critical:** `teleopPeriodic()` silently parses game-specific message `'B'`/`'R'` to
set ACTIVE/INACTIVE. The alliance-flip logic and `hasData()` guard are completely
undocumented. Also missing class and constructor Javadoc.

**Tasks:**
- Class Javadoc: explain `TimedRobot`, what this class is responsible for
- Constructor: explain DataLogManager, PortForwarder, health timer setup
- `robotPeriodic()`: document the 10 Hz rate-limit reason (NT4 flood prevention)
- `teleopPeriodic()`: explain game-specific message format, `'B'`/`'R'` semantics, why alliance determines ACTIVE vs INACTIVE
- Lifecycle stubs: `disabledInit/Periodic/Exit`, `autonomousInit/Periodic/Exit`, `testInit/Periodic/Exit`, `simulationPeriodic`

**File:** `src/main/java/frc/robot/Robot.java`

---

### D1-4 `RunHopper.java`

**Why critical:** Two silent behaviors — a 4-loop (80 ms) counter delay before any output,
and a `!isActive() && zone == ALLIANCE` no-op condition — that are completely undocumented.
No class Javadoc.

**Tasks:**
- Class Javadoc: explain purpose, relationship to `ShootWhenReady` and `StaggerHopper`
- Constructor: document parameters
- `execute()`: explain `counter > 3` delay (settle time) and the state/zone guard
- `initialize()` / `end()` / `isFinished()`: brief docs

**File:** `src/main/java/frc/robot/commands/RunHopper.java`

---

### D1-5 `Vision.java`

**Why critical:** `periodic()` contains Kalman filter updates, 4 m measurement rejection,
and distance-scaled standard deviations. Inline comments reference cryptic `// H-2`,
`// H-3`, `// M-4` codes with no legend. No class Javadoc.

**Tasks:**
- Class Javadoc: explain multi-camera fusion, Kalman filter role, replay mode
- No-arg constructor: document that it disables all pose estimation (`isReplay = true`)
- `periodic()`: document each pipeline stage — estimate fetch, 4 m rejection threshold, std-dev scaling
- `simulationPeriodic()`: explain the hardcoded 5° rotation and the TODO for turret angle
- `setUpSim()`, `getEstimatedPose()`, `resetVisionPose()`: brief docs
- Replace `// H-2`, `// H-3`, `// M-4` comments with self-contained explanations

**File:** `src/main/java/frc/robot/subsystems/vision/Vision.java`

---

## D-2 — Priority 2: Public API Gaps

These are interfaces and subsystems where missing docs prevent safe reuse.

### D2-1 `ShooterIO.java` + `ShooterIOHardware.java` + `ShooterIOSim.java`

**Tasks:**
- `ShooterIO`: class Javadoc explaining IO layer pattern and available implementations
- `ShooterIOInputs`: class Javadoc; explain the `@AutoLog` annotation behavior
- `setVelocity(AngularVelocity, boolean)`: document `useRecovery` — switches between `VelocityTorqueCurrentFOC` (normal) and `VelocityDutyCycle` (recovery), why the distinction matters
- `ShooterIOHardware`: class Javadoc, constructor, `setVelocity()` change-detection optimization
- `ShooterIOSim`: class Javadoc, constructor, `kAlpha` field Javadoc

**Files:** `src/main/java/frc/robot/subsystems/shooter/ShooterIO*.java`

---

### D2-2 `OnboardLogger.java`

**Tasks:**
- Class Javadoc: explain the static registry pattern and that `logAll()` must be called each loop
- Constructor: document the self-registration side effect
- `logAll()`: prominent Javadoc — without this call no data is written
- All `register*()` methods: namespace semantics, lazy-write behavior
- `registerMeasurement()`: document the null-safety + `DriverStation.reportError()` path

**File:** `src/main/java/frc/robot/util/OnboardLogger.java`

---

### D2-3 `Intake.java`

**Tasks:**
- Class Javadoc: describe roller vs. deploy motors, open-loop operation
- `deployIntake()`: clarify it sets deploy arm motor speed (not a boolean deploy trigger)
- `setIntakeSpeed()`, `periodic()`: brief docs

**File:** `src/main/java/frc/robot/subsystems/intake/Intake.java`

---

### D2-4 `AimMeasurement.java`

**Tasks:**
- Class Javadoc: explain role as calibration data record, how it feeds `ToFAim`
- All fields: units and meaning — especially `shooterControl` (wheel RPS from calibration table)

**File:** `src/main/java/frc/robot/aiming/AimMeasurement.java`

---

### D2-5 `TurretIOSim.java`

**Tasks:**
- Class Javadoc: explain simulation model, first-order lag, SmartDashboard calibration hook
- `updateInputs()`: document `"Turret/Successful Calibration?"` SmartDashboard read
- `setPosition()` / `calibrate()`: brief docs

**File:** `src/main/java/frc/robot/subsystems/turret/TurretIOSim.java`

---

## D-3 — Priority 3: Moderate Gaps

### D3-1 `TurretIODisabled.java`
- Class Javadoc: when and why to use the no-op implementation

### D3-2 `TuneAim.java`
- Class Javadoc: SmartDashboard-driven manual tuning, when to activate this strategy

### D3-3 `CommandSwerveDrivetrain.java`
- `startSimThread()`: explain 5 ms / 200 Hz Notifier and why it's required for Phoenix 6 sim
- `configureAutoBuilder()`: explain why PathPlanner uses `RobotStateMachine.getPose()` not CTRE's own pose
- `periodic()`: note that CTRE internal odometry handles updates; empty override is intentional
- Supplier methods: `rotationSupplier()`, `modulePositionsSupplier()`, `poseSupplier()`, `getPigeon()`, `getAngularVel()`

**File:** `src/main/java/frc/robot/subsystems/drivetrain/CommandSwerveDrivetrain.java`

### D3-4 `TurretIOHardware.java`
- Class Javadoc + constructor: follower setup, MotionMagic `DynamicMotionMagicVoltage` rationale
- `updateInputs()`: explain why motor encoder is used post-calibration instead of CANcoder

### D3-5 `RunIntake.java`
- Class Javadoc + all methods
- `execute()`: explain why `deployIntake(0.15)` is hardcoded (hold arm deployed) vs. the configurable roller speed

### D3-6 `ControllerRumble.java`
- Class Javadoc + all methods
- `execute()` vs `end()` asymmetry: left-only set, both-zero clear — explain why

### D3-7 `StaggerHopper.java`
- Class Javadoc: explain "stagger" concept, 0.35 s / 0.2 s timing, when used

### D3-8 `StatusSignalUtil.java`
- `registerRioSignals()` / `registerCANivoreSignals()`: explain bulk signal registration
- `refreshAll()`: document bulk-refresh requirement and when to call it

### D3-9 `ShooterConstants.java`
- Class Javadoc + inner class Javadocs
- `kMotorConfig`: explain `PeakReverseDutyCycle(0)` (one-direction-only)
- `scoringMeasurements` / `feedingMeasurements`: explain format and distinction

### D3-10 `VisionIO.java` + `VisionEstimate.java` + `PhotonVisionIO.java` + `PhotonVisionSimIO.java`
- `VisionEstimate`: constructor + null-return behavior of `getPose()` / `getTimestamp()`
- `PhotonVisionIO.getLatestResult()`: caching strategy
- `getBestRange()`: document the zeroed camera/target height arguments and their implication
- `PhotonVisionSimIO`: `mountedOnTurret()`, `isMountedOnTurret()`, `getCameraSim()`

**Files:** `src/main/java/frc/robot/subsystems/vision/VisionIO.java`, `VisionEstimate.java`; `subsystems/vision/photonvision/PhotonVisionIO.java`, `PhotonVisionSimIO.java`

### D3-11 `AimStrategy.java`
- `update()`: document the contract (must not return null, must fill `status`, when to return `impossible()`)

### D3-12 `Main.java` + `Constants.java` inner classes
- `Main`: one-line class Javadoc
- `Constants` inner classes: `RobotConstants`, `DriveConstants`, `ModuleConstants`, `OIConstants`, `AutoConstants`, `MotorConstants`, `VisionConstants`, `TurretConstants` — each needs a class Javadoc explaining what the group covers

### D3-13 `SysIDUtil.java` + `ShooterValuesSenable.java`
- `SysIDUtil`: document stubs and when they'll be implemented (Stage 8)
- `ShooterValuesSenable`: confirm if live code or dead; document or delete

**Files:** `src/main/java/frc/robot/subsystems/drivetrain/SysIDUtil.java`; `src/main/java/frc/robot/subsystems/shooter/ShooterValuesSenable.java`

---

## Commit Strategy

One commit per D-1 file (they are large and complex).
D-2 and D-3 can be batched by related subsystem.

| Commit tag | Files |
|------------|-------|
| `docs-d1-physicsaim` | `PhysicsAim.java` |
| `docs-d1-statemachine` | `RobotStateMachine.java` |
| `docs-d1-robot` | `Robot.java` |
| `docs-d1-runhopper` | `RunHopper.java` |
| `docs-d1-vision` | `Vision.java` |
| `docs-d2-shooter` | `ShooterIO.java`, `ShooterIOHardware.java`, `ShooterIOSim.java` |
| `docs-d2-logger-intake-aim` | `OnboardLogger.java`, `Intake.java`, `AimMeasurement.java`, `TurretIOSim.java` |
| `docs-d3-commands` | `RunIntake.java`, `ControllerRumble.java`, `StaggerHopper.java` |
| `docs-d3-drivetrain` | `subsystems/drivetrain/CommandSwerveDrivetrain.java`, `TurretIOHardware.java`, `TurretIODisabled.java` |
| `docs-d3-vision` | `subsystems/vision/VisionIO.java`, `VisionEstimate.java`, `photonvision/PhotonVisionIO.java`, `PhotonVisionSimIO.java` |
| `docs-d3-aiming` | `TuneAim.java`, `AimStrategy.java` |
| `docs-d3-constants` | `Constants.java` inner classes, `ShooterConstants.java` |
| `docs-d3-util` | `util/StatusSignalUtil.java`, `subsystems/drivetrain/SysIDUtil.java`, `subsystems/shooter/ShooterValuesSenable.java` |

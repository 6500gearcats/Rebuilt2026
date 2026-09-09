# Rebuilt2026 — FRC Robot Code

**Team:** GearCats 6500 | **Season:** 2026 FIRST Robotics Competition
**Framework:** WPILib Command-Based (Java 17) | **Branch:** `leto`

Every fact below was verified against source on 2026-09-09. If something here looks wrong,
it's more likely stale than the surrounding docs — see
[`plans/PLANNING_GUIDE.md`](plans/PLANNING_GUIDE.md) for how to keep it in sync.

---

## Table of Contents

1. [What This Robot Does](#what-this-robot-does)
2. [Project Structure](#project-structure)
3. [WPILib Primer for New Developers](#wpilib-primer-for-new-developers)
4. [Subsystems](#subsystems)
5. [Commands](#commands)
6. [Robot State Machine](#robot-state-machine)
7. [Vision & Pose Estimation](#vision--pose-estimation)
8. [Operator Interface & Controls](#operator-interface--controls)
9. [Autonomous (PathPlanner)](#autonomous-pathplanner)
10. [Hardware Reference](#hardware-reference)
11. [Vendor Libraries](#vendor-libraries)
12. [Building & Deploying](#building--deploying)
13. [Tuning & Telemetry](#tuning--telemetry)

---

## What This Robot Does

A game piece ("fuel") shooting robot with an auto-aiming turret, a flywheel-and-hood
shooter, floor intake, and a two-stage hopper. There is **no climber** on this robot — one
existed in an earlier design and was fully removed (Stage 3 of
[`plans/Rebuilt2026_RefactorPlan.md`](plans/Rebuilt2026_RefactorPlan.md)).

Key capabilities:
- **Swerve drive** — full holonomic motion, CTRE TalonFX-based (Phoenix 6 / Tuner X generated)
- **Auto-aiming turret** — single-axis yaw mechanism, tracks the hub while the robot drives
- **Physics- and calibration-based aiming** — a ballistic solver (`PhysicsAim`) and an
  empirical lookup-table solver (`ToFAim`) both implement a common `AimStrategy` interface;
  `LeadCompensator` adjusts the aim point for robot motion during ball flight
- **Flywheel + hood shooter** — two flywheel motors, a separately-actuated hood for launch
  angle, closed-loop velocity and position control
- **Intake & hopper** — floor pickup, indexed and fed into the shooter
- **Multi-camera PhotonVision** — AprilTag detection fused into a Kalman-filter pose estimate
- **Game-phase state machine** — tracks match time and alliance to know when shooting is
  allowed, drives LED feedback

---

## Project Structure

```
Rebuilt2026/
├── src/main/java/frc/robot/
│   ├── Robot.java                  # WPILib entry point — do not add blocking calls here
│   ├── RobotContainer.java         # Wires subsystems, commands, controller bindings, autos
│   ├── RobotStateMachine.java      # Singleton: pose, aiming pipeline, alliance, LED state
│   ├── Constants.java              # Hardware IDs, physical measurements, tuning values
│   ├── Main.java                   # Framework bootstrap — do not modify
│   ├── aiming/                     # AimStrategy, PhysicsAim, ToFAim, LeadCompensator, AimParams
│   ├── commands/                   # AimPrep, ShootWhenReady, RunIntake, RunHopper,
│   │                                #   StaggerHopper, ControllerRumble
│   ├── superstructure/
│   │   └── StateManager.java       # Bridges Turret/Shooter to RobotStateMachine
│   ├── subsystems/
│   │   ├── drivetrain/             # CommandSwerveDrivetrain, Telemetry, SysIDUtil
│   │   ├── shooter/                # Shooter, ShooterIO(+Hardware/Sim), ShooterConstants
│   │   ├── turret/                 # Turret, TurretIO(+Hardware/Sim/Disabled), TurretConstants
│   │   ├── hopper/Hopper.java
│   │   ├── intake/Intake.java
│   │   └── vision/                 # Vision, VisionIO, LocalizationConstants, photonvision/
│   ├── util/                       # OnboardLogger, StatusSignalUtil
│   └── generated/                  # TunerConstants*.java — Tuner X generated, never hand-edit
├── src/main/deploy/pathplanner/    # Autonomous path + auto files (PathPlanner GUI)
├── vendordeps/                     # Vendor library JSON files
├── plans/                          # Active/historical planning docs — see plans/README.md
└── build.gradle
```

There is **no top-level `vision/` or `utility/` package** — both were consolidated into
`subsystems/` (`pkg_plan.md`). There is **no `Flywheel.java`, `RangeFinder.java`,
`Climber.java`, or `LedCANdle.java`** — all deleted; see the Subsystems section below for
what replaced the first two.

---

## WPILib Primer for New Developers

WPILib's **Command-Based** paradigm is the architecture this project uses.

**Subsystems** represent a physical mechanism (drivetrain, shooter, intake, ...). Each:
- Extends `SubsystemBase`
- Has a `periodic()` method the scheduler calls every 20 ms (50 Hz), in any robot mode
- Owns its motors and sensors — should be the only place that calls motor APIs directly

**Commands** are actions the robot performs. Each declares which subsystems it
`addRequirements()`s so the scheduler prevents two commands from fighting over one
mechanism, and implements `initialize()` / `execute()` / `isFinished()` / `end(interrupted)`.

**The scheduler** (`CommandScheduler.getInstance().run()`, called from `Robot.robotPeriodic()`)
each loop: refreshes bulk CTRE signals, polls triggers/buttons, runs `execute()` on active
commands, removes finished ones, then calls `periodic()` on every subsystem.

**Default commands** run whenever nothing else requires that subsystem — the drivetrain's
default command is field-centric joystick driving.

**`RobotContainer`** is where every subsystem is instantiated, every button binding is made,
and the autonomous chooser is built. **`Robot.java`** is the WPILib entry point; you rarely
need to touch it directly.

---

## Subsystems

### 1. Drivetrain — `subsystems/drivetrain/CommandSwerveDrivetrain.java`

CTRE Phoenix 6 swerve, generated by Tuner X (`generated/TunerConstants2.java`). Extends the
generated `TunerSwerveDrivetrain` (itself `SwerveDrivetrain<TalonFX, TalonFX, CANcoder>`) and
implements WPILib's `Subsystem` so it participates in the scheduler.

- 4 modules (front-left, front-right, back-left, back-right), each a TalonFX drive motor +
  TalonFX steer motor + CANcoder — see [Hardware Reference](#hardware-reference) for IDs
- Pigeon2 IMU for heading
- `SwerveRequest.FieldCentric` default command reads the driver joystick
- In simulation, a 200 Hz `Notifier` calls `updateSimState()` — required for CTRE's sim
  physics to look realistic; guarded by `Utils.isSimulation()`, does not run on hardware
- PathPlanner `AutoBuilder` is configured here, using `RobotStateMachine.getPose()` (the
  Kalman-fused estimate) rather than the drivetrain's own raw odometry
- SysId characterization routines exist for translation, steer, and rotation, gated behind
  `SysIDUtil` (currently a stub — real implementation is Stage 8, once hardware exists)

### 2. Turret — `subsystems/turret/Turret.java`

Single-axis yaw mechanism, ±0.5 rotation (±180°) travel, enforced by TalonFX soft limits.
One TalonFX motor + one absolute CANcoder — **a single encoder, no dual-encoder CRT math**
(an earlier Hackbots design used two encoders combined via Chinese-Remainder-style
resolution; this design's encoder reads the angle directly).

- `TurretIO` interface with three implementations: `TurretIOHardware` (real CAN),
  `TurretIOSim` (first-order lag simulation), `TurretIODisabled` (no-op, used when no mode
  matches)
- `MotionMagic` (`DynamicMotionMagicVoltage`) drives smooth position moves
- `findCC()` (closest-congruent) handles multi-turn wrapping so the turret takes the short
  way to any target angle rather than unwinding through zero
- `track(StateManager)` continuously aims at the hub; `home()` / `forwards()` drive to fixed
  references; `jog(rotationsPerSecond)` is the manual override

### 3. Shooter — `subsystems/shooter/Shooter.java`

Two flywheel TalonFX motors (motor 2 follows motor 1) plus a separate hood TalonFX with its
own CANcoder for launch angle.

- `ShooterIO` interface: `ShooterIOHardware` (real CAN) / `ShooterIOSim` (first-order lag)
- Normal shooting uses `VelocityTorqueCurrentFOC`; a "recovery mode"
  (`VelocityDutyCycle`) kicks in when speed error exceeds a threshold, trading accuracy for
  lower current draw / faster recovery after a ball passes through
- `shoot(Supplier<AimParams>)` drives both wheel speed and hood angle every loop from the
  aiming pipeline's output; wheels are explicitly commanded to zero on disable and on
  command end so they don't coast indefinitely

### 4. Hopper — `subsystems/hopper/Hopper.java`

Two TalonFX motors, driven directly (no IO layer — open-loop duty cycle is sufficient here):
indexer (belt, advances balls toward the kicker) and kicker (final launch into the flywheel).
`startAllMotors(hopperSpeed, kickerSpeed)` / `stopAllMotors()` are the two entry points;
`RunHopper`/`StaggerHopper`/`ShootWhenReady` commands drive them.

### 5. Intake — `subsystems/intake/Intake.java`

Two TalonFX motors, driven directly: roller (spins to pull balls in) and deploy (extends the
intake arm — held at a constant duty cycle against gravity, not position-controlled).

### 6. Vision — `subsystems/vision/Vision.java`

**PhotonVision only.** Limelight support was fully deprecated and deleted (Stage 1) — there
is no Limelight code anywhere in this repo. `VisionIO` interface with two implementations:
`PhotonVisionIO` (real camera, `MULTI_TAG_PNP_ON_COPROCESSOR` pose strategy) and
`PhotonVisionSimIO` (synthetic frames from `VisionSystemSim`, `LOWEST_AMBIGUITY` strategy —
the sim estimator doesn't run on a coprocessor thread).

- **Hardware** currently wires two named PhotonVision cameras: `Thrifty_cam_1`,
  `Thrifty_cam_2` (see `RobotContainer`'s `REAL` case for exact mount offsets)
- **Simulation** wires one synthetic camera named `"photonvision"`
- Odometry propagation + vision fusion runs every loop on hardware; in simulation,
  `Vision.periodic()` is a no-op (CTRE's own sim thread already tracks ground-truth
  odometry, so the Kalman filter would add nothing but loop-time cost — see
  `plans/project_sim-loop-overruns` context in the AI session memory, or just the comment at
  the top of `Vision.periodic()`)
- A `SwerveDrivePoseEstimator` fuses odometry with vision; distance-scaled standard
  deviations trust closer detections more; measurements more than 4 m from the current
  estimate are rejected outright

### 7. LEDs — removed

`LedCANdle.java` does not exist anywhere in this codebase. If LED feedback is wanted again,
it needs to be rebuilt — `RobotStateMachine` still contains the color/blink *logic*
(`newPostedValue()`, the `switching`/`switchingRed`/`switchingGreen` flags), but nothing
currently drives a physical LED device from it.

### 8. Climber — removed

No climber exists in code or (per Stage 3) in the robot design. `Climber.java`,
`ClimbPole.java`, and all associated bindings/`NamedCommands` were deleted.

---

## Commands

Current roster (`src/main/java/frc/robot/commands/`) — six files, all current:

| Command | What it does |
|---|---|
| `AimPrep` | Runs turret tracking (`Turret.track`) and shooter spin-up (`Shooter.shoot`) in parallel until interrupted. Does not feed the hopper. |
| `ShootWhenReady` | `waitUntil(RobotStateMachine.isShootReady)` then runs the hopper via `startEnd` (guarantees motors stop on interrupt). Designed to run in parallel with `AimPrep`. |
| `RunIntake` | Sets intake roller + deploy motor duty cycles; runs while the bound button is held. |
| `RunHopper` | Feeds the hopper after a 4-loop (~80 ms) settle delay, with a state/zone guard that blocks output when the robot is inactive in its own alliance zone. |
| `StaggerHopper` | `RunHopper` for 0.35 s, then a 0.2 s pause — gives the flywheel time to recover speed between consecutive shots. |
| `ControllerRumble` | Haptic feedback — rumbles a controller's left motor while scheduled, zeroes both motors on end. |

**Superseded/deleted commands** (do not look for these — they don't exist):
`ShootingSequenceUTS`, `AlignTurretToHub`, `UpToSpeedHopperShoot`, `MoveTurret`,
`SetTurretAngle`, `ShootFuel`, `ShootingSequence`, `CoolSnurbo`, `UncoolSnurbo`, `BurstFire`,
`ClimbPole` — all removed in Stage 5 (Hackbots integration) or Stage 3 (climber removal).
`RobotContainer.aimAndShoot()` (a private helper, not a standalone command file) is the
modern equivalent of the old `ShootingSequenceUTS` — it's `Commands.parallel(AimPrep,
ShootWhenReady)`.

---

## Robot State Machine

**File:** `RobotStateMachine.java` — an eagerly-initialized singleton
(`RobotStateMachine.getInstance()`), owning the `Shooter` subsystem instance itself (not
`RobotContainer`), the aiming pipeline, pose caching, and LED-pattern logic.

### States

| State | Meaning |
|---|---|
| `ACTIVE` | Scoring window is open — robot may shoot |
| `INACTIVE` | Scoring window is closed — robot should not shoot |

Transitions are driven by `getState()`, which reads match time + the FMS game-specific
message and follows a hardcoded alliance-dependent schedule (see the method's own Javadoc
for the full timing table) — this also drives a red/green/white LED blink pattern intended
for a physical LED device that no longer exists in code (see [LEDs](#7-leds--removed)).

### Field Zones (`checkZone()`)

| Zone | X range |
|---|---|
| `ALLIANCE` | x < 5.4 m (mirrored by alliance) |
| `NEUTRAL_TOP` | 5.4–11.0 m, y > 4.2 m |
| `NEUTRAL_CENTER` | 5.4–11.0 m, 3.8–4.2 m |
| `NEUTRAL_BOTTOM` | 5.4–11.0 m, y < 3.8 m |
| `OPPONENT` | x > 11.0 m (mirrored by alliance) |

### Aiming Pipeline (`getAimParams()`)

```
getAimParams()
  → LeadCompensator.computeLeadTarget(hub, turretPose, fieldVelocity, ToFAim)  [5-iter]
  → ToFAim.update(virtualTarget, turretPose, kZero)  [final params: yaw, pitch, output, tof]
```

`ToFAim` is a calibration-table-based strategy (`ShooterConstants.scoringMeasurements`);
`PhysicsAim` is an alternative first-principles ballistic solver — both implement the common
`AimStrategy` interface and are interchangeable.

---

## Vision & Pose Estimation

See [Subsystems → Vision](#6-vision--subsystemsvisionvisionjava) above for the code-level
detail. Cameras currently wired in `RobotContainer`: `Thrifty_cam_1`, `Thrifty_cam_2` on
hardware; one synthetic `"photonvision"` camera in simulation.

---

## Operator Interface & Controls

Two Xbox controllers. Bindings live in `RobotContainer.configureBindings()`.

### Driver (port 0)

| Input | Action |
|---|---|
| Left stick | Field-centric translation (Y = forward/back, X = strafe) |
| Right stick X | Rotation |
| Start | Re-zero field-centric forward to current heading |
| Left bumper (hold) | Run intake rollers |
| Right bumper (hold) | Reverse shooter wheels (clear jams) |
| Y | Home the turret |

### Gunner (port 1)

| Input | Action |
|---|---|
| Right trigger (>10%) | Intake sequence — deploy arm briefly, then run rollers |
| Left trigger (>10%, hold) | `AimPrep` (turret track + shooter spin-up) + driver rumble |
| Left bumper (hold) | `ShootWhenReady` — feed hopper once on-target |
| X | Home the turret |
| POV right (90°) | Jog turret clockwise, 0.5 rot/s |
| POV left (270°) | Jog turret counter-clockwise, 0.5 rot/s |

When disabled, the drivetrain runs a `SwerveRequest.Idle` so the robot can be pushed by hand.

---

## Autonomous (PathPlanner)

**Library:** PathPlanner 2026.1.2. Paths/autos live in `src/main/deploy/pathplanner/`.

### Current auto files

`Artemis2`, `FinalHailMary`, `FinalHope`, `HumanPlayer`, `ModifiedWakeRight`,
`ProjectHailMaryRight`, `WakeCenter`, `WakeLeft`, `simple center`, `WakeRight`,
`SimonAutoTest`.

### NamedCommands (registered in `RobotContainer`'s constructor)

| Name | Command |
|---|---|
| `IntakeFuel` | `RunIntake` at −1, no timeout |
| `IntakeFuelJason` | `RunIntake` at −1, 5 s timeout |
| `Intake` | `RunIntake` at −0.1, 0.2 s timeout |
| `IntakeLong` | `RunIntake` at −0.1, 0.8 s timeout |
| `ShootFuel`, `ShootFuel3s/5s/7s/10s` | `aimAndShoot()` (`AimPrep` ∥ `ShootWhenReady`), timed variants withTimeout |
| `NewShootFuel3s/4s/5s/8s/10s` | Same as above — kept as separate names for existing auto files |
| `ManualShootFuel3s` | `ShootWhenReady` only (no turret tracking), 3 s timeout |
| `TrenchStartAngle` | `Turret.home()` |
| `AlignTurret`, `AlignTurret1s` | `Turret.track()` only (no shooting), timed variant |
| `BopBop` | Deploy intake briefly (−0.3, 0.35 s), then run rollers (−1, 0.3 s) |
| `SpeedUp` | `Commands.none()` — stub, marked TODO Stage 6 in source |

**No climb-related `NamedCommands` exist** — `ClimbUp2s`, `ClimbDown2s`, and `Climb` were
removed along with the climber (Stage 3); no auto file references them.

To add a new named command: `NamedCommands.registerCommand("Name", command)` in
`RobotContainer`'s constructor, **before** `AutoBuilder.buildAutoChooser()` is called.

---

## Hardware Reference

### CAN Bus IDs (all on the roboRIO bus — no CANivore in use)

| ID | Device | Role |
|---|---|---|
| 0 | TalonFX | Swerve front-left drive |
| 1 | TalonFX | Swerve front-right drive |
| 2 | TalonFX | Swerve back-right drive |
| 3 | TalonFX | Swerve back-left drive |
| 4 | TalonFX | Swerve front-left steer |
| 5 | TalonFX | Swerve front-right steer |
| 6 | TalonFX | Swerve back-right steer |
| 7 | TalonFX | Swerve back-left steer |
| 8 | CANcoder | Swerve front-left encoder |
| 9 | CANcoder | Swerve front-right encoder |
| 10 | CANcoder | Swerve back-right encoder |
| 11 | CANcoder | Swerve back-left encoder |
| 20 | TalonFX | Intake roller |
| 21 | TalonFX | Intake deploy |
| 22 | TalonFX | Hopper indexer |
| 23 | TalonFX | Hopper kicker |
| 30 | Pigeon2 | Swerve IMU |
| 31 | TalonFX | Shooter flywheel motor 2 |
| 32 | TalonFX | Shooter hood motor |
| 33 | CANcoder | Shooter hood encoder |
| 34 | TalonFX | Turret motor |
| 35 | CANcoder | Turret encoder |
| 36 | TalonFX | Shooter flywheel motor 1 |

IDs 20–23 are wired for real hardware already. **IDs 30 (Pigeon aside), 31–36 are
placeholders** pending Stage 8-1 CAN assignment on the assembled robot — see
`ShooterConstants`/`TurretConstants` comments. `Constants.MotorConstants` also carries three
unused legacy stub IDs (`kTurretYawMotorID=12`, `kShooterMotorRightID=13`,
`kShooterMotorLeftID=14`) from an earlier Hackbots configuration — nothing in current
hardware IO reads them; don't reuse those numbers without deleting the stubs first.

CAN ID 25 (an earlier climber) and CAN ID 50 (an earlier CANdle) are free — neither device
exists in code.

### Digital I/O

None currently used. The turret has no limit switch in this design — it calibrates from its
absolute CANcoder instead (see [Turret](#2-turret--subsystemsturretturretjava)).

### Network Ports

| Port | Service |
|---|---|
| 5800 | PhotonVision web UI, forwarded to `photonvision.local` (`Robot.java`'s `PortForwarder`) |
| 5810 | NetworkTables 4 (SmartDashboard / AdvantageScope / Shuffleboard) — WPILib's NT4 default; not set explicitly anywhere in this codebase. (Not 1735 — that's the older NT3 port; the previous version of this doc had it wrong.) |

### Team Number

**6500**, set in `.wpilib/wpilib_preferences.json`.

---

## Vendor Libraries

| Library | Vendor | Actually used for |
|---|---|---|
| Phoenix6 (26.1.0) | CTRE | TalonFX motors, CANcoders, Pigeon 2 — the entire drivetrain, shooter, turret, intake, hopper |
| PathplannerLib (2026.1.2) | PathPlanner | Autonomous path following |
| PhotonLib (v2026.1.1-rc-3, release candidate) | PhotonVision | AprilTag detection — see `plans/SIM_PROGRESS.md` for a known, deliberately-kept deprecated API call pending a stable PhotonVision release |
| WPILibNewCommands | WPILib | Command-based framework |
| REVLib | REV Robotics | **Legacy/unused for hardware control.** `Constants.DriveConstants` still imports `SparkBaseConfig.IdleMode` for a handful of unused constants left over from an earlier REV SPARK MAX swerve design. The actual swerve motors today are CTRE TalonFX. Do not add new REV hardware without confirming this vendordep still needs to exist. |

To update a vendor library: WPILib VS Code extension → **WPILib: Manage Vendor Libraries →
Check for Updates (online)**.

---

## Building & Deploying

**Prerequisites:** WPILib 2026 suite (Java 17, Gradle, VS Code extensions); roboRIO
connected via USB, Ethernet, or radio for deploy.

```bash
./gradlew build          # compile only, no robot required
./gradlew deploy         # deploy to the roboRIO
./gradlew simulateJava   # launch desktop simulation
```

PowerShell compiles need the WPILib JDK explicitly:
```powershell
$env:JAVA_HOME = "C:\Users\Public\wpilib\2026\jdk"
```

Simulation opens the WPILib sim GUI, which includes the simulated Driver Station (enable
mode, joystick assignment) — see the **Joysticks** panel to assign a physical controller to
a port. AdvantageScope, connected to `localhost`, is the recommended tool for live plots and
field visualization; drag `DriveState/Pose` into a 2D field view to see the robot. (This
topic is published by `subsystems/drivetrain/Telemetry.java`, wired up in `RobotContainer`
as of 2026-09-09 — before that fix this instruction did not work, because the class existed
but was never instantiated. See `plans/review_plan.md` R1-A3 if this ever regresses.)

---

## Tuning & Telemetry

**Durable, per-loop telemetry** (voltage, current, temperature, accumulated energy for every
motor) is written directly to the `.wpilog` file via `util/OnboardLogger.java` — not
`SmartDashboard`. This avoids NT4 bandwidth cost and survives past a live session for
post-match review in AdvantageScope. See `plans/logging_plan.md` for the full per-motor
breakdown; every TalonFX on the robot (16 total: 8 drivetrain, 3 shooter, 1 turret, 2
intake, 2 hopper) is covered.

`OnboardLogger.logAll()` and `StatusSignalUtil.refreshAll()` are both called once per loop
from `Robot.robotPeriodic()` — if you add a new `OnboardLogger` registration or a new
`StatusSignalUtil.registerRioSignals()` call, it only takes effect because those two calls
already exist; you don't need to add anything to `Robot.java` yourself.

**Slower-moving values** still go to `SmartDashboard` at a rate-limited 10 Hz (not every
loop) via `Timer.advanceIfElapsed(0.1)` guards — e.g. `Robot/BatteryVoltageV`,
`Robot/FieldZone`, `Aiming/LeadOffsetXM/YM`. This pattern exists specifically to avoid the
loop overruns documented in Stage 0 of `plans/Rebuilt2026_RefactorPlan.md`.

**Aim pipeline calibration:** `ShooterConstants.scoringMeasurements` /
`feedingMeasurements` are `AimMeasurement` records (distance, hood pitch, flywheel speed,
time-of-flight) collected empirically on real hardware and interpolated at runtime by
`ToFAim`. Re-validate before trusting them — they're carried over from an earlier
(Hackbots) robot's calibration and need re-measurement on this chassis (Stage 8-8).

**SysId:** routines exist on `CommandSwerveDrivetrain` (translation/steer/rotation) and are
stubbed via `SysIDUtil` for the turret and shooter — real characterization is Stage 8, once
hardware exists to characterize.

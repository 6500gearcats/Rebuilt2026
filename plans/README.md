# Rebuilt2026 — FRC Robot Code

**Team:** GearCats | **Season:** 2026 FIRST Robotics Competition
**Game:** Reefscape | **Framework:** WPILib Command-Based (Java 17)

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

This robot is built for a game where robots shoot game pieces (fuel/balls) into a target called the **Hub**, collect game pieces off the floor, and optionally climb a pole at the end of the match.

Key capabilities:
- **Swerve Drive** — the robot can move in any direction without rotating first (holonomic)
- **Auto-aiming Turret** — a rotating turret keeps the shooter aimed at the Hub while the robot drives
- **Flywheel Shooter** — spins up to a calculated speed based on distance to the Hub, then fires
- **Intake & Hopper** — collects game pieces from the floor and indexes them into the shooter
- **Multi-Camera Vision** — fuses April Tag detections from four cameras to know exactly where the robot is on the field
- **Game-Phase State Machine** — automatically tracks match time and alliance color to know when and where to shoot

---

## Project Structure

```
Rebuilt2026/
├── src/main/java/frc/robot/
│   ├── Robot.java                  # Entry point; WPILib calls periodic methods here
│   ├── RobotContainer.java         # Wires subsystems, commands, and controller bindings
│   ├── Constants.java              # Hardware IDs, physical measurements, tuning values
│   ├── RobotStateMachine.java      # Game logic: alliance, match phase, field zones
│   ├── commands/                   # Actions the robot can take
│   │   ├── AlignTurretToHub.java
│   │   ├── ShootingSequenceUTS.java
│   │   ├── UpToSpeedHopperShoot.java
│   │   ├── MoveTurret.java
│   │   ├── SetTurretAngle.java
│   │   ├── RunIntake.java
│   │   ├── CoolSnurbo.java / UncoolSnurbo.java
│   │   └── ClimbPole.java
│   ├── subsystems/
│   │   ├── CommandSwerveDrivetrain.java   # Swerve drive + odometry
│   │   ├── vision/Vision.java             # Camera fusion & pose estimation
│   │   ├── LedCANdle.java                 # LED strip controller
│   │   ├── Climber.java                   # End-game climber (partially implemented)
│   │   ├── hopper/Hopper.java             # Indexer & kicker motors
│   │   ├── intake/Intake.java             # Floor intake mechanism
│   │   └── turret/
│   │       ├── Turret.java                # Yaw rotation motor
│   │       └── Flywheel.java             # Shooter motors
│   └── utility/
│       └── RangeFinder.java              # Distance-to-speed lookup table
├── src/main/deploy/
│   └── pathplanner/                      # Autonomous path files
├── vendordeps/                           # Third-party library JSON files
├── build.gradle                          # Gradle build configuration
└── .wpilib/wpilib_preferences.json       # Team number & deployment settings
```

---

## WPILib Primer for New Developers

WPILib is the official framework for FRC robots. This project uses the **Command-Based** paradigm, which is the recommended architecture. Here are the core concepts:

### Subsystems

A **Subsystem** represents a physical mechanism on the robot (drivetrain, shooter, intake, etc.). Each subsystem:
- Extends `SubsystemBase`
- Has a `periodic()` method that runs every 20ms (50 Hz) during any robot mode
- Owns the motors and sensors for that mechanism
- Should be the *only* place that directly calls motor APIs

Think of subsystems as nouns — things the robot has.

### Commands

A **Command** is an action the robot performs. Each command:
- Extends `CommandBase` (or uses factory methods)
- Declares which subsystems it `requires()` — WPILib prevents two commands from fighting over the same subsystem
- Has four lifecycle methods:
  - `initialize()` — runs once when the command starts
  - `execute()` — runs every 20ms while the command is active
  - `isFinished()` — returns `true` when the command should stop
  - `end(boolean interrupted)` — cleanup when the command finishes or is cancelled

Think of commands as verbs — things the robot does.

### The Scheduler

WPILib's `CommandScheduler` runs every loop cycle. It:
1. Polls all triggers and buttons
2. Starts any commands whose trigger condition became true
3. Calls `execute()` on all running commands
4. Removes commands where `isFinished()` returns `true`
5. Calls `periodic()` on all registered subsystems

### Default Commands

Each subsystem can have a **default command** that runs whenever no other command requires that subsystem. For example, the drivetrain's default command reads the joystick and drives the robot. When an autonomous command takes over the drivetrain, the default command is pre-empted.

### RobotContainer

`RobotContainer.java` is where everything is wired together:
- Subsystem instances are created here
- Button bindings (`whenPressed`, `whileTrue`, etc.) are configured here
- The autonomous command chooser is set up here

### Robot.java

`Robot.java` is the WPILib entry point. It calls `RobotContainer` at startup and delegates to WPILib's scheduler in each periodic method. You rarely need to modify `Robot.java` directly.

---

## Subsystems

### 1. CommandSwerveDrivetrain

**File:** `subsystems/CommandSwerveDrivetrain.java`

This is the swerve drive base. Swerve drive gives the robot full holonomic motion — it can translate in any direction while independently controlling its rotation.

**Hardware:**
- 4 drive motors (REV SPARK MAX, IDs 1–4) — spin the wheels to create forward motion
- 4 steer motors (REV SPARK MAX, IDs 5–8) — rotate each wheel module to any angle
- Pigeon 2 IMU — measures the robot's heading (yaw)

**Key configuration:**
- Wheel diameter: 3 inches (0.0762 m)
- Drive gear ratio: ~6.75:1
- Wheelbase: 25.5 inches square
- Max speed: 4.6 m/s

**Control modes:**
- **Teleoperated:** Field-centric control (left stick = translation relative to the field, right stick = rotation). The robot automatically flips the reference frame for Red vs. Blue alliance.
- **Autonomous:** Accepts `ChassisSpeeds` (x, y, omega) from PathPlanner or any command.
- **SysId:** Characterization routines are built in for tuning feedforward gains.

**Odometry:** The drivetrain maintains an odometry estimate from wheel encoders + gyro. Vision corrections are applied via the `Vision` subsystem.

---

### 2. Turret

**File:** `subsystems/turret/Turret.java`

The turret is a single-axis rotation stage that points the shooter at the Hub while the robot drives.

**Hardware:**
- TalonFX motor (ID 12)
- Digital limit switch (DIO pin 4) — used to find the zero position on startup

**Motion range:** -110° to +110° from center (55 rotations of motor = full travel). The turret physically cannot rotate 360°.

**Homing:** On startup, the turret uses a `SetTurretAngle` command to drive toward the limit switch. When the switch is triggered, the motor encoder is zeroed. All subsequent positions are relative to that home.

**Control:** CTRE `PositionVoltage` with Motion Magic (PID + feedforward). Three PID slots are configured; Slot 2 is currently active:
- kS = 0.2 (static friction overcome voltage)
- kV = 13 (velocity feedforward)
- kA = 5 (acceleration feedforward)
- kP = 6 (position proportional)
- kD = 1 (derivative damping)

**Safety:** The turret checks bounds before commanding movement and ignores out-of-range requests unless the driver explicitly enables override mode.

---

### 3. Flywheel

**File:** `subsystems/turret/Flywheel.java`

The flywheel shooter spins up to speed and then launches game pieces when the hopper feeds them.

**Hardware:**
- TalonFX right motor (ID 13) — primary motor
- TalonFX left motor (ID 14) — follows right motor in the opposite direction (counter-rotation for backspin/topspin as required)

**Control:** CTRE `VelocityVoltage` in rotations per second (RPS). The sensor ratio is 0.6 (gear reduction between motor and wheel).

**PID (Slot 0):**
- kS = 0.3087, kV = 0.076456, kA = 0.010904, kP = 0.026467

**Speed modes:**
- **Normal:** Speed is looked up from `RangeFinder` based on distance to Hub
- **Snurbo:** Speed reduced to 15% for close-range or controlled shots
- **Trench boost:** Automatically increases to 70+ RPS when robot is under the trench structure

**"Up to speed" check:** Before firing, `isUpToSpeed()` confirms the flywheel is within 2 RPS of the target. The hopper will not run until this is satisfied.

---

### 4. Hopper

**File:** `subsystems/hopper/Hopper.java`

The hopper indexes game pieces from the collector into the flywheel.

**Hardware:**
- TalonFX indexer motor (ID 22) — conveyor that moves pieces up from intake
- TalonFX kicker motor (ID 23) — final kick to feed the piece into the spinning flywheel

**Typical operation:**
- Indexer: -0.9 duty cycle (inward direction)
- Kicker: +1.0 duty cycle (toward flywheel)

Both motors run only when the flywheel is up to speed (commanded by `UpToSpeedHopperShoot`).

---

### 5. Intake

**File:** `subsystems/intake/Intake.java`

The intake collects game pieces from the floor.

**Hardware:**
- TalonFX intake motor (ID 20) — spins rollers to pull pieces in
- TalonFX deploy motor (ID 21) — extends the intake arm out from the frame

**Typical operation:**
- Deploy at 0.15–0.3 duty cycle to extend
- Intake at -1 to -3 duty cycle to collect (negative = inward)
- Deploy at 0 to hold extended position

---

### 6. Vision

**File:** `subsystems/vision/Vision.java`

See [Vision & Pose Estimation](#vision--pose-estimation) section below for full details.

---

### 7. LedCANdle

**File:** `subsystems/LedCANdle.java`

Controls the LED strip for driver feedback and team spirit.

**Hardware:** CTRE CANdle (ID 50) with 177-LED strip + 8 onboard LEDs

**Patterns available:**
- Solid color (any RGB)
- Rainbow animation
- Pride flag patterns: trans, bi, american, GearCats team colors
- Brightness scaling

LEDs are also used by `RobotStateMachine` to signal game state changes (e.g., flash white when switching from ACTIVE to INACTIVE).

---

### 8. Climber (Partial)

**File:** `subsystems/Climber.java`

End-game climber for scoring points by ascending a pole. Currently stubbed out — the motor (TalonFX ID 25) and limit switch (DIO 4) are declared but the subsystem is disabled.

**Planned commands:**
- `ClimbPole(climber, +0.5)` — climb up (Y button)
- `ClimbPole(climber, -0.5)` — climb down (A button)

---

## Commands

### ShootingSequenceUTS

**File:** `commands/ShootingSequenceUTS.java`

The full shooting pipeline, run as a `ParallelCommandGroup`:

1. **UpToSpeedHopperShoot** — ramp the flywheel to the `RangeFinder`-calculated speed, then start the hopper when the speed is confirmed
2. **AlignTurretToHub** — continuously calculate the angle from the turret to the Hub and command the turret there
3. **CoolSnurbo** — enable reduced-speed mode during the shot sequence

All three run simultaneously. The sequence ends when the trigger is released (these commands are bound to a trigger held condition).

---

### AlignTurretToHub

**File:** `commands/AlignTurretToHub.java`

The most complex command in the codebase. Every 20ms it:

1. Gets the robot's current pose from `RobotStateMachine`
2. Gets the Hub's field position (from April Tag 10 for Red, Tag 20 for Blue)
3. Accounts for the turret's physical offset from the robot center (0.1524m x, 0.0635m y)
4. Predicts where the Hub will be when the shot arrives (using robot velocity × flight time)
5. Calculates the angle from the turret to the predicted Hub position
6. Subtracts the robot's current heading to get a turret-relative angle
7. Bounds-checks the result and commands the turret to that angle

The shot flight time is calculated from ballistics:
- Launch angle: ~65°
- Height difference: 56.375" − 19" = 37.375"

This command returns `false` from `isFinished()` and runs continuously while the trigger is held.

---

### UpToSpeedHopperShoot

**File:** `commands/UpToSpeedHopperShoot.java`

1. Reads the target speed from `SmartDashboard` ("Shoot Speed") or from `RangeFinder`
2. Sets the flywheel to that speed
3. Waits (loops in `execute()`) until `flywheel.isUpToSpeed()` returns true
4. Starts hopper (indexer + kicker) to feed the game piece into the flywheel

---

### MoveTurret

**File:** `commands/MoveTurret.java`

Manual turret control. Accepts a `DoubleSupplier` (e.g., from a D-pad axis) and applies it as a speed command every loop cycle. Used for manual fine-tuning when auto-aim is not available.

---

### RunIntake

**File:** `commands/RunIntake.java`

Sets the intake and deploy motors to the specified speeds. Runs continuously while the bound button is held (default command behavior with `.whileTrue()`).

---

### CoolSnurbo / UncoolSnurbo

**Files:** `commands/CoolSnurbo.java`, `commands/UncoolSnurbo.java`

Toggle commands that set the flywheel's "snurbo" flag. When snurbo is active, the flywheel runs at 15% speed regardless of the range-based target. Useful for close-range shots where full speed would overshoot.

---

## Robot State Machine

**File:** `RobotStateMachine.java`

This is a singleton (one global instance) that acts as the central hub for all game logic. Any subsystem or command can call `RobotStateMachine.getInstance()` to get the current state.

### States

| State | Meaning |
|-------|---------|
| `ACTIVE` | Robot is in a zone/phase where it should be shooting |
| `INACTIVE` | Robot should not shoot (wrong zone, waiting period) |

### Responsibilities

- **Alliance tracking:** Reads FMS alliance color (Red/Blue) at match start
- **Pose tracking:** Owns the `SwerveDrivePoseEstimator` (Kalman filter); aggregates odometry from the drivetrain and vision corrections from cameras
- **Field zone detection:** Calculates which zone of the field the robot is in based on its X/Y pose
- **Phase scheduling:** Switches between ACTIVE/INACTIVE at specific match timestamps (different schedules for Red and Blue alliance phases)
- **Hub location:** Determines the Hub pose from the relevant April Tag (10 = Red, 20 = Blue) offset by -0.58m
- **LED signaling:** Triggers LED patterns when state changes

### Field Zones

| Zone | X range (Blue perspective) |
|------|---------------------------|
| ALLIANCE | x < 5.4m |
| NEUTRAL_TOP | 5.4 < x < 11.0, y < 3.8 |
| NEUTRAL_CENTER | 5.4 < x < 11.0, 3.8 < y < 4.2 |
| NEUTRAL_BOTTOM | 5.4 < x < 11.0, y > 4.2 |
| OPPONENT | x > 11.0m |

(Boundaries mirror for Red alliance.)

---

## Vision & Pose Estimation

**File:** `subsystems/vision/Vision.java`

### Why Pose Estimation?

Wheel odometry alone drifts over time. By fusing odometry with April Tag detections, the robot always knows where it is on the field — which is critical for auto-aiming the turret accurately.

### Hardware

| Camera | Type | Purpose |
|--------|------|---------|
| Thrifty_cam_1 | PhotonVision USB | April Tag detection (front-left) |
| Thrifty_cam_2 | PhotonVision USB | April Tag detection (front-right) |
| limelight-gcd | Limelight | April Tag detection |
| limelight-gcc | Limelight | April Tag detection |

All cameras are mounted near the front of the robot at approximately (0.254m, 0.254m, 0.2032m) from the robot center, angled upward at 62° pitch.

### How It Works

1. Each camera detects April Tags and reports the tag ID and robot-relative pose
2. WPILib's `SwerveDrivePoseEstimator` maintains a Kalman filter:
   - Prediction step: uses wheel encoder velocities + gyro (runs every 20ms)
   - Update step: incorporates each camera measurement when one arrives
3. The filter weights each source by its configured standard deviation (uncertainty)
4. The resulting `Pose2d` (x, y, rotation) is published to NetworkTables and used by `AlignTurretToHub` and `RobotStateMachine`

### Standard Deviations (Tuning Knobs)

Lower values = more trust in that source.
- State (odometry): 0.1m x, 0.1m y, 0.1° θ
- Vision corrections: 0.1m x, 0.1m y, 0.1° θ

If vision is noisy, increase vision std devs. If odometry drifts, decrease them.

---

## Operator Interface & Controls

Two Xbox controllers are used.

### Driver Controller (Port 0 — "Pranav")

| Input | Action |
|-------|--------|
| Left Stick Y | Drive forward / backward |
| Left Stick X | Strafe left / right |
| Right Stick X | Rotate robot |
| Left Bumper (hold) | Run intake at -3 speed |
| Right Bumper (hold) | Enable Snurbo (15% flywheel speed) |
| Left Trigger (hold) | Full shooting sequence (aim + spin up + fire) |
| Right Trigger (press) | Deploy intake + brief intake burst |
| Start | Reset field-centric heading (re-zero gyro to current direction) |
| A Button | Auto-align turret to Hub |
| Y Button | Zero turret (run homing routine) |
| Back | Toggle turret bounds override |
| D-Pad Right | Turret manual +0.2 speed |
| D-Pad Left | Turret manual -0.2 speed |

> **Tip for new drivers:** "Reset heading" (Start button) should be pressed when the robot is physically facing the direction you want to call "forward." This re-synchronizes the field-centric control if the gyro has drifted.

---

## Autonomous (PathPlanner)

**Library:** PathPlanner 2026.1.2

Autonomous paths are created in the PathPlanner GUI app and saved as JSON files under `src/main/deploy/pathplanner/`. The robot replays these paths during the autonomous period.

### Named Commands

PathPlanner paths reference named commands that are registered in `RobotContainer`:

| Name | What it does |
|------|-------------|
| `IntakeFuel` | Run intake briefly |
| `IntakeLong` | Run intake for a longer duration |
| `ShootFuel` | Run the full shooting sequence |
| `AlignTurret` | Aim turret at Hub |
| `SpeedUp` | Ramp flywheel to speed |
| `Climb` | Trigger climb sequence |
| `ClimbUp2s` | Climb up for 2 seconds |
| `ClimbDown2s` | Climb down for 2 seconds |
| `BopBop` | Custom choreography |

To add a new named command, register it in `RobotContainer.java`:
```java
NamedCommands.registerCommand("MyCommand", new MyCommand(subsystem));
```

---

## Hardware Reference

### CAN Bus IDs

| ID | Device | Role |
|----|--------|------|
| 1–4 | SPARK MAX | Swerve drive motors |
| 5–8 | SPARK MAX | Swerve steer motors |
| 12 | TalonFX | Turret yaw |
| 13 | TalonFX | Flywheel right |
| 14 | TalonFX | Flywheel left |
| 20 | TalonFX | Intake roller |
| 21 | TalonFX | Intake deploy |
| 22 | TalonFX | Hopper indexer |
| 23 | TalonFX | Hopper kicker |
| 25 | TalonFX | Climber (disabled) |
| 50 | CANdle | LED controller |

### Digital I/O (DIO)

| Pin | Device |
|-----|--------|
| 4 | Turret home limit switch |

### Network Ports

| Port | Service |
|------|---------|
| 5800 | PhotonVision web UI (forwarded to photonvision.local) |
| 1735 | NetworkTables (SmartDashboard, Shuffleboard) |

---

## Vendor Libraries

Vendor libraries extend WPILib with support for specific hardware. They are checked in as JSON files in `vendordeps/` and downloaded automatically by Gradle.

| Library | Vendor | What it enables |
|---------|--------|-----------------|
| Phoenix6 | CTRE | TalonFX motors, CANdle LEDs, Pigeon 2 IMU |
| REVLib | REV Robotics | SPARK MAX motor controllers |
| PhotonLib | PhotonVision | PhotonVision camera API |
| PathplannerLib | PathPlanner | Autonomous path following |
| WPILibNewCommands | WPILib | Command-based framework extensions |

To update a vendor library, use the WPILib VS Code extension: **WPILib > Manage Vendor Libraries > Check for Updates (online)**.

---

## Building & Deploying

### Prerequisites
- WPILib 2026 suite (includes Java 17, Gradle, VS Code extensions)
- roboRIO connected via USB, Ethernet, or radio

### Build only (no robot required)
```bash
./gradlew build
```

### Deploy to robot
```bash
./gradlew deploy
```
Or use the WPILib VS Code button: **Deploy Robot Code** (Ctrl+Shift+P → "WPILib: Deploy Robot Code").

### Run simulation
```bash
./gradlew simulateJava
```
Simulation opens a GUI where you can drive the robot on a virtual field, inject joystick inputs, and watch NetworkTables. PhotonVision cameras are simulated with realistic noise.

### Team number
Configured in `.wpilib/wpilib_preferences.json`. Change it there if deploying to a different RoboRIO.

---

## Tuning & Telemetry

### SmartDashboard / Shuffleboard

The robot publishes numerous values to NetworkTables that appear in SmartDashboard or Shuffleboard:

| Key | What it shows |
|-----|--------------|
| `Shoot Speed` | Current flywheel target speed (RPS) — writable for manual override |
| `Turret Angle` | Current turret angle in degrees |
| `Robot Pose` | X, Y, heading on the field |
| `Flywheel Speed` | Actual vs. requested flywheel speed |
| `Alliance` | Detected alliance color |
| `Robot State` | ACTIVE or INACTIVE |
| `Field Zone` | Current zone (ALLIANCE, NEUTRAL_*, OPPONENT) |

### RangeFinder (Distance-to-Speed Lookup)

**File:** `utility/RangeFinder.java`

Uses `InterpolatingDoubleTreeMap` to linearly interpolate shooter speed from measured distance-to-Hub data points:

| Distance (m) | Speed (RPS) |
|-------------|-------------|
| 2.23 | 50 |
| 2.80 | 57 |
| 4.20 | 81 |
| 5.20 | 100 |

To add a new data point, add a line in `RangeFinder.java`:
```java
map.put(distanceMeters, speedRPS);
```

Recalibrate this table whenever the shooter geometry changes.

### SysId (System Identification)

The drivetrain has built-in SysId routines for characterizing feedforward gains. These are triggered via controller button bindings in simulation or on-robot testing mode. See the WPILib SysId documentation for the full workflow.

### PID Tuning

All PID values are in `Constants.java` or in the subsystem constructors. After changing a value, rebuild and deploy. For faster iteration, CTRE motors support live tuning via Phoenix Tuner X connected over USB or CAN.

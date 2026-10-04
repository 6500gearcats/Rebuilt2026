# Graph Report - Rebuilt2026  (2026-10-03)

## Corpus Check
- 88 files · ~48,539 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 1023 nodes · 2125 edges · 52 communities (43 shown, 9 thin omitted)
- Extraction: 93% EXTRACTED · 7% INFERRED · 0% AMBIGUOUS · INFERRED: 157 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `0ad03cac`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- Pose2d
- Turret
- PhotonVisionIO
- Vision
- RobotStateMachine
- PhotonVisionSimIO
- LimelightHelpers
- Constants
- Phoenix Tuner X: A Beginner's Guide to Configuring an FRC Robot
- Flywheel
- Hopper
- Robot
- LimelightIO
- Telemetry
- CommandSwerveDrivetrain
- RobotContainer.java
- .updateTargetPose
- WPILib License
- Pose3d
- ShootingSequence.java
- Leto → leto_main Branch Integration Plan
- Command
- Intake
- .getLimelightNTTableEntry
- RobotStateMachine.java
- LedCANdle
- PoseEstimate
- AlignTurretToHub
- ShotSafety
- Shooting calibration
- ShooterValuesSenable
- .RobotContainer
- MoveTurret
- .getState
- Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options...
- Q: Help me understand a good procedure for calibrating TOF
- HomeIntake
- Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls
- Q: Turn measured hood endpoints into soft limits and encoder zero settings
- Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?
- Deployment Example Instructions
- Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?
- Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings
- gradlew
- Main
- Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?
- Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO
- Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration
- UpToSpeedHopperShootTest.java
- LimelightTarget_Barcode

## God Nodes (most connected - your core abstractions)
1. `LimelightHelpers` - 104 edges
2. `RobotStateMachine` - 89 edges
3. `Turret` - 47 edges
4. `Flywheel` - 35 edges
5. `RobotContainer` - 34 edges
6. `CommandSwerveDrivetrain` - 32 edges
7. `PhotonVisionIO` - 32 edges
8. `PhotonVisionSimIO` - 30 edges
9. `Vision` - 28 edges
10. `Hopper` - 25 edges

## Surprising Connections (you probably didn't know these)
- `Robot` --references--> `RobotContainer`  [EXTRACTED]
  src/main/java/frc/robot/Robot.java → src/main/java/frc/robot/RobotContainer.java
- `Robot` --references--> `RobotStateMachine`  [EXTRACTED]
  src/main/java/frc/robot/Robot.java → src/main/java/frc/robot/RobotStateMachine.java
- `RobotContainer` --references--> `RobotStateMachine`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/RobotStateMachine.java
- `RobotContainer` --references--> `Climber`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/subsystems/Climber.java
- `RobotContainer` --references--> `CommandSwerveDrivetrain`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java

## Import Cycles
- None detected.

## Hyperedges (group relationships)
- **WPILib Redistribution Conditions** — wpilib_license_wpilib_license, wpilib_license_redistribution_and_use, wpilib_license_source_form, wpilib_license_binary_form, wpilib_license_copyright_notice, wpilib_license_disclaimer [EXTRACTED 1.00]

## Communities (52 total, 9 thin omitted)

### Community 0 - "Pose2d"
Cohesion: 0.14
Nodes (6): Pose2d, LimelightResults, LimelightTarget_Classifier, LimelightTarget_Detector, LimelightTarget_Fiducial, LimelightTarget_Retro

### Community 1 - "Turret"
Cohesion: 0.15
Nodes (9): DigitalInput, CANcoder, ControlRequest, Override, Pose3d, PositionVoltage, TalonFX, TalonFXConfiguration (+1 more)

### Community 2 - "PhotonVisionIO"
Cohesion: 0.10
Nodes (17): PhotonCamera, PhotonTrackedTarget, EstimatedRobotPose, Field2d, Matrix, N1, N3, Override (+9 more)

### Community 3 - "Vision"
Cohesion: 0.20
Nodes (11): AprilTagFieldLayout, Field2d, N3, Override, Pose2d, Rotation2d, StructPublisher, SwerveModulePosition (+3 more)

### Community 4 - "RobotStateMachine"
Cohesion: 0.16
Nodes (4): Alliance, Pose2d, XboxController, RobotStateMachine

### Community 5 - "PhotonVisionSimIO"
Cohesion: 0.07
Nodes (20): PhotonCameraSim, SimCameraProperties, Matrix, N1, N3, Override, PhotonPipelineResult, PhotonPoseEstimator (+12 more)

### Community 7 - "Constants"
Cohesion: 0.11
Nodes (23): Constraints, IdleMode, RobotConfig, AutoConstants, ClimberConstants, Constants, DriveConstants, GyroConstants (+15 more)

### Community 8 - "Phoenix Tuner X: A Beginner's Guide to Configuring an FRC Robot"
Cohesion: 0.05
Nodes (37): A device doesn't appear in Tuner, Actually moving a motor, Before you start, CAN bus utilization, Decide this first: Tuner or code?, Device shows as not licensed when it should be, Faults, and why sticky ones matter, Firmware or version mismatch errors (+29 more)

### Community 9 - "Flywheel"
Cohesion: 0.06
Nodes (24): CANcoderConfiguration, InterpolatingDoubleTreeMap, Slot0Configs, CoolSnurbo, Override, Flywheel, ControlRequest, Override (+16 more)

### Community 10 - "Hopper"
Cohesion: 0.08
Nodes (15): SequentialCommandGroup, Override, RunHopper, Override, RunHopperBack, StaggerHopper, UncoolSnurbo, FieldZone (+7 more)

### Community 11 - "Robot"
Cohesion: 0.19
Nodes (5): Command, Override, Timer, Robot, TimedRobot

### Community 12 - "LimelightIO"
Cohesion: 0.21
Nodes (3): Override, Rotation2d, LimelightIO

### Community 13 - "Telemetry"
Cohesion: 0.20
Nodes (15): DoubleArrayPublisher, DoublePublisher, Mechanism2d, MechanismLigament2d, NetworkTableInstance, ChassisSpeeds, NetworkTable, Pose2d (+7 more)

### Community 14 - "CommandSwerveDrivetrain"
Cohesion: 0.05
Nodes (44): Angle, ApplyRobotSpeeds, CANBus, ClosedLoopOutputType, Current, Distance, DriveMotorArrangement, LinearVelocity (+36 more)

### Community 15 - "RobotContainer.java"
Cohesion: 0.15
Nodes (12): CommandPS4Controller, FieldCentric, PointWheelsAt, SendableChooser, SlewRateLimiter, Command, CommandXboxController, Pose3d (+4 more)

### Community 16 - ".updateTargetPose"
Cohesion: 0.05
Nodes (15): CalibrationInputs, ShotCalibration, TrialSnapshot, Pose2d, Translation2d, ShotPlanner, ShotSample, ShotSettings (+7 more)

### Community 17 - "WPILib License"
Cohesion: 0.15
Nodes (15): As-Is Software, Binary Form, Copyright Notice, Damage Exclusion, License Disclaimer, Endorsement Restriction, FIRST, Liability Disclaimer (+7 more)

### Community 20 - "Leto → leto_main Branch Integration Plan"
Cohesion: 0.06
Nodes (31): 0.1 Scope and direction, 0.2 This is a planning document only, 0.3 Process conventions to carry into execution, 0.4 Reference points, 0. Ground rules, 1. Why this is not a normal `git merge`, 2.1 Bucket A — `leto`-only additions (accept as-is; `leto_main` has no equivalent), 2.2 Bucket B — `leto_main`-only additions (must be ported onto `leto`'s tree) (+23 more)

### Community 21 - "Command"
Cohesion: 0.07
Nodes (14): Command, GenericHID, ClimbPole, Override, ControllerRumble, Override, Override, SetTurretAngle (+6 more)

### Community 22 - "Intake"
Cohesion: 0.17
Nodes (6): Override, RunIntake, MotorConstants, Intake, PositionVoltage, TalonFX

### Community 23 - ".getLimelightNTTableEntry"
Cohesion: 0.12
Nodes (6): DoubleArrayEntry, NetworkTableEntry, ObjectMapper, NetworkTable, RawDetection, URL

### Community 24 - "RobotStateMachine.java"
Cohesion: 0.12
Nodes (11): BooleanSupplier, Color, TurretConstants, ChassisSpeeds, CommandXboxController, Field2d, Pose3d, StructPublisher (+3 more)

### Community 25 - "LedCANdle"
Cohesion: 0.18
Nodes (10): CANdle, CANdleConfiguration, EmptyAnimation, RainbowAnimation, RGBWColor, DoubleSupplier, Override, Timer (+2 more)

### Community 28 - "ShotSafety"
Cohesion: 0.23
Nodes (3): ShotSafety, Test, ShotSafetyTest

### Community 29 - "Shooting calibration"
Cohesion: 0.20
Nodes (9): Accepted-row provenance, Before shooting, Controls and dashboard, Efficient collection session, How interpolation works, Moving-shot validation, Shooting calibration, Troubleshooting (+1 more)

### Community 30 - "ShooterValuesSenable"
Cohesion: 0.24
Nodes (4): Sendable, SendableBuilder, Override, ShooterValuesSenable

### Community 32 - "MoveTurret"
Cohesion: 0.33
Nodes (3): DoubleSupplier, Override, MoveTurret

### Community 33 - ".getState"
Cohesion: 0.36
Nodes (3): RobotState, ACTIVE, INACTIVE

### Community 34 - "Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options..."
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options..., Source Nodes

### Community 35 - "Q: Help me understand a good procedure for calibrating TOF"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Help me understand a good procedure for calibrating TOF, Source Nodes

### Community 36 - "HomeIntake"
Cohesion: 0.21
Nodes (4): HomeIntake, Override, IntakeConstants, Override

### Community 37 - "Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls, Source Nodes

### Community 38 - "Q: Turn measured hood endpoints into soft limits and encoder zero settings"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Turn measured hood endpoints into soft limits and encoder zero settings, Source Nodes

### Community 39 - "Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?, Source Nodes

### Community 40 - "Deployment Example Instructions"
Cohesion: 0.40
Nodes (6): Deploy Directory, Deployment Example Instructions, Filesystem.getDeployDirectory, Home Folder, RoboRIO, WPILib Function

### Community 41 - "Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?, Source Nodes

### Community 42 - "Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings, Source Nodes

### Community 43 - "gradlew"
Cohesion: 0.83
Nodes (3): gradlew script, die(), warn()

### Community 47 - "Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?, Source Nodes

### Community 48 - "Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO, Source Nodes

### Community 49 - "Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration, Source Nodes

## Knowledge Gaps
- **115 isolated node(s):** `REAL`, `SIM`, `OIConstants`, `NeoMotorConstants`, `GyroConstants` (+110 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **9 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Work-memory lessons

**Preferred sources** — corroborated by past sessions; start here.
- `Turret` (6× useful, score=4.653978507) _(code changed — re-verify)_
- `RobotContainer` (5× useful, score=3.894263178) _(code changed — re-verify)_
- `Constants` (5× useful, score=3.863357088)
- `TunerConstants` (3× useful, score=2.343026467)
- `Flywheel` (3× useful, score=2.205587587) _(code changed — re-verify)_
- `Vision` (3× useful, score=2.197152057)
- `AlignTurretToHub` (3× useful, score=2.060869265) _(code changed — re-verify)_
- `RobotStateMachine` (3× useful, score=2.060869265) _(code changed — re-verify)_
- `PhotonVisionIO` (2× useful, score=1.582276987) _(code changed — re-verify)_
- `UpToSpeedHopperShoot` (2× useful, score=1.270247847) _(code changed — re-verify)_

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `RobotStateMachine` connect `RobotStateMachine` to `MoveTurret`, `.getState`, `Turret`, `Vision`, `Flywheel`, `Hopper`, `Robot`, `CommandSwerveDrivetrain`, `RobotContainer.java`, `.updateTargetPose`, `ShootingSequence.java`, `RobotStateMachine.java`, `AlignTurretToHub`, `.RobotContainer`?**
  _High betweenness centrality (0.184) - this node is a cross-community bridge._
- **Why does `LimelightHelpers` connect `LimelightHelpers` to `Pose2d`, `LimelightIO`, `RobotContainer.java`, `Pose3d`, `LimelightTarget_Barcode`, `.getLimelightNTTableEntry`, `PoseEstimate`?**
  _High betweenness centrality (0.166) - this node is a cross-community bridge._
- **Why does `RobotContainer` connect `RobotContainer.java` to `Turret`, `PhotonVisionIO`, `Vision`, `RobotStateMachine`, `Flywheel`, `Hopper`, `Robot`, `Telemetry`, `CommandSwerveDrivetrain`, `ShootingSequence.java`, `Command`, `Intake`, `LedCANdle`, `.RobotContainer`?**
  _High betweenness centrality (0.090) - this node is a cross-community bridge._
- **What connects `REAL`, `SIM`, `OIConstants` to the rest of the system?**
  _115 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `Pose2d` be split into smaller, more focused modules?**
  _Cohesion score 0.14039408866995073 - nodes in this community are weakly interconnected._
- **Should `PhotonVisionIO` be split into smaller, more focused modules?**
  _Cohesion score 0.10034013605442177 - nodes in this community are weakly interconnected._
- **Should `PhotonVisionSimIO` be split into smaller, more focused modules?**
  _Cohesion score 0.06836055656382335 - nodes in this community are weakly interconnected._
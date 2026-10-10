# Graph Report - Rebuilt2026  (2026-10-10)

## Corpus Check
- 73 files · ~40,062 words
- Verdict: corpus is large enough that graph structure adds value.

## Summary
- 871 nodes · 1854 edges · 47 communities (41 shown, 6 thin omitted)
- Extraction: 94% EXTRACTED · 6% INFERRED · 0% AMBIGUOUS · INFERRED: 106 edges (avg confidence: 0.8)
- Token cost: 0 input · 0 output

## Graph Freshness
- Built from commit: `341426f0`
- Run `git rev-parse HEAD` and compare to check if the graph is stale.
- Run `graphify update .` after code changes (no API cost).

## Community Hubs (Navigation)
- PhotonVisionSimIO
- TunerConstants
- Turret
- PhotonVisionIO
- LimelightHelpers
- Phoenix Tuner X: A Beginner's Guide to Configuring an FRC Robot
- CommandSwerveDrivetrain
- Intake
- Pose2d
- Constants
- .getState
- RunHopper
- LimelightIO
- Pose3d
- Robot
- RobotContainer.java
- Vision
- LedCANdle
- .periodic
- Telemetry
- Flywheel
- RobotStateMachine
- PoseEstimate
- .getLimelightNTDoubleArray
- .periodic
- Q: Can you check to see if the vision subsystem is currently working in this branch and describe the cameras and pose estimation?
- .getLimelightNTTableEntry
- Q: Why does gunner D-pad up flywheel speed jump back to the initial speed?
- ShooterValuesSenable
- ControllerRumble
- Command
- Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options...
- Q: Help me understand a good procedure for calibrating TOF
- Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls
- Q: Turn measured hood endpoints into soft limits and encoder zero settings
- Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?
- Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?
- Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings
- Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?
- Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO
- Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration
- gradlew
- Main
- LimelightTarget_Barcode

## God Nodes (most connected - your core abstractions)
1. `LimelightHelpers` - 104 edges
2. `RobotStateMachine` - 77 edges
3. `Turret` - 45 edges
4. `Flywheel` - 40 edges
5. `RobotContainer` - 36 edges
6. `CommandSwerveDrivetrain` - 32 edges
7. `PhotonVisionIO` - 32 edges
8. `PhotonVisionSimIO` - 30 edges
9. `Vision` - 28 edges
10. `Hopper` - 26 edges

## Surprising Connections (you probably didn't know these)
- `Robot` --references--> `RobotContainer`  [EXTRACTED]
  src/main/java/frc/robot/Robot.java → src/main/java/frc/robot/RobotContainer.java
- `Robot` --references--> `RobotStateMachine`  [EXTRACTED]
  src/main/java/frc/robot/Robot.java → src/main/java/frc/robot/RobotStateMachine.java
- `RobotContainer` --references--> `RobotStateMachine`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/RobotStateMachine.java
- `RobotContainer` --references--> `CommandSwerveDrivetrain`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
- `RobotContainer` --references--> `Intake`  [EXTRACTED]
  src/main/java/frc/robot/RobotContainer.java → src/main/java/frc/robot/subsystems/intake/Intake.java

## Import Cycles
- None detected.

## Communities (47 total, 6 thin omitted)

### Community 0 - "PhotonVisionSimIO"
Cohesion: 0.07
Nodes (20): PhotonCameraSim, SimCameraProperties, Matrix, N1, N3, Override, PhotonPipelineResult, PhotonPoseEstimator (+12 more)

### Community 1 - "TunerConstants"
Cohesion: 0.07
Nodes (30): Angle, CANBus, CANcoderConfiguration, ClosedLoopOutputType, Current, DriveMotorArrangement, LinearVelocity, MomentOfInertia (+22 more)

### Community 2 - "Turret"
Cohesion: 0.06
Nodes (22): AlignTurretToHub, Override, Pose2d, DoubleSupplier, Override, MoveTurret, Override, SetTurretAngle (+14 more)

### Community 3 - "PhotonVisionIO"
Cohesion: 0.10
Nodes (17): PhotonCamera, PhotonTrackedTarget, EstimatedRobotPose, Field2d, Matrix, N1, N3, Override (+9 more)

### Community 5 - "Phoenix Tuner X: A Beginner's Guide to Configuring an FRC Robot"
Cohesion: 0.05
Nodes (37): A device doesn't appear in Tuner, Actually moving a motor, Before you start, CAN bus utilization, Decide this first: Tuner or code?, Device shows as not licensed when it should be, Faults, and why sticky ones matter, Firmware or version mismatch errors (+29 more)

### Community 6 - "CommandSwerveDrivetrain"
Cohesion: 0.07
Nodes (20): ApplyRobotSpeeds, Notifier, Pigeon2, CommandSwerveDrivetrain, Command, Direction, Matrix, N1 (+12 more)

### Community 7 - "Intake"
Cohesion: 0.11
Nodes (10): HomeIntake, Override, Override, RunIntake, IntakeConstants, MotorConstants, Intake, Override (+2 more)

### Community 9 - "Constants"
Cohesion: 0.12
Nodes (23): Constraints, DigitalInput, IdleMode, RobotConfig, AutoConstants, ClimberConstants, Constants, DriveConstants (+15 more)

### Community 10 - ".getState"
Cohesion: 0.24
Nodes (3): RobotState, ACTIVE, INACTIVE

### Community 11 - "RunHopper"
Cohesion: 0.18
Nodes (4): Override, RunHopper, Override, RunHopperBack

### Community 12 - "LimelightIO"
Cohesion: 0.21
Nodes (3): Override, Rotation2d, LimelightIO

### Community 13 - "Pose3d"
Cohesion: 0.13
Nodes (5): Pose3d, LimelightResults, LimelightTarget_Classifier, LimelightTarget_Detector, LimelightTarget_Fiducial

### Community 14 - "Robot"
Cohesion: 0.14
Nodes (5): Command, Override, Timer, Robot, TimedRobot

### Community 15 - "RobotContainer.java"
Cohesion: 0.05
Nodes (27): CommandPS4Controller, FieldCentric, ParallelCommandGroup, PointWheelsAt, SendableChooser, SequentialCommandGroup, SlewRateLimiter, ClimbPole (+19 more)

### Community 16 - "Vision"
Cohesion: 0.20
Nodes (11): AprilTagFieldLayout, Field2d, N3, Override, Pose2d, Rotation2d, StructPublisher, SwerveModulePosition (+3 more)

### Community 17 - "LedCANdle"
Cohesion: 0.18
Nodes (11): CANdle, CANdleConfiguration, EmptyAnimation, RainbowAnimation, RGBWColor, DoubleSupplier, Override, Timer (+3 more)

### Community 18 - ".periodic"
Cohesion: 0.19
Nodes (3): Override, UpToSpeedHopperShoot, Override

### Community 19 - "Telemetry"
Cohesion: 0.20
Nodes (15): DoubleArrayPublisher, DoublePublisher, Mechanism2d, MechanismLigament2d, NetworkTableInstance, ChassisSpeeds, NetworkTable, Pose2d (+7 more)

### Community 20 - "Flywheel"
Cohesion: 0.21
Nodes (5): Flywheel, ControlRequest, TalonFX, TalonFXConfiguration, VelocityVoltage

### Community 21 - "RobotStateMachine"
Cohesion: 0.15
Nodes (10): Alliance, Color, Distance, ChassisSpeeds, CommandXboxController, Pose2d, Pose3d, StructPublisher (+2 more)

### Community 22 - "PoseEstimate"
Cohesion: 0.21
Nodes (3): PoseEstimate, RawDetection, RawFiducial

### Community 25 - "Q: Can you check to see if the vision subsystem is currently working in this branch and describe the cameras and pose estimation?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Can you check to see if the vision subsystem is currently working in this branch and describe the cameras and pose estimation?, Source Nodes

### Community 26 - ".getLimelightNTTableEntry"
Cohesion: 0.14
Nodes (5): DoubleArrayEntry, NetworkTableEntry, ObjectMapper, NetworkTable, URL

### Community 27 - "Q: Why does gunner D-pad up flywheel speed jump back to the initial speed?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Why does gunner D-pad up flywheel speed jump back to the initial speed?, Source Nodes

### Community 28 - "ShooterValuesSenable"
Cohesion: 0.24
Nodes (4): Sendable, SendableBuilder, Override, ShooterValuesSenable

### Community 29 - "ControllerRumble"
Cohesion: 0.36
Nodes (3): GenericHID, ControllerRumble, Override

### Community 32 - "Command"
Cohesion: 0.13
Nodes (12): Command, InterpolatingDoubleTreeMap, DoubleSupplier, Override, ShootFuel, FieldZone, ALLIANCE, NEUTRAL_BOTTOM (+4 more)

### Community 34 - "Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options..."
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: when we are testing the auto, for the first part the shots are consistently extremely off, the shoots are about 3 feet to the left of the hub, it seems like the robot locks on and accuracy dramatically improves once the cameras are able to see tags on the hub. what's going on here ? give me some options..., Source Nodes

### Community 35 - "Q: Help me understand a good procedure for calibrating TOF"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Help me understand a good procedure for calibrating TOF, Source Nodes

### Community 36 - "Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Enable the new hood motor and turret and hood CANcoders with four manual trim controls, Source Nodes

### Community 37 - "Q: Turn measured hood endpoints into soft limits and encoder zero settings"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Turn measured hood endpoints into soft limits and encoder zero settings, Source Nodes

### Community 38 - "Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Can the hood limits use TalonFXConfiguration SoftwareLimitSwitchConfigs and how do kMaxAngle and kMinAngle work?, Source Nodes

### Community 39 - "Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: After configuring TalonFX remote CANcoder soft limits, can the hood limit checks be removed from periodic?, Source Nodes

### Community 40 - "Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Calibrate the new turret absolute encoder and configure TalonFX firmware soft limits from measured forward, left, and right readings, Source Nodes

### Community 41 - "Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Where are SmartDashboard values published, what do they report, and which subsystem or command owns each publication?, Source Nodes

### Community 42 - "Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: Diagnose OpenJDK Client VM native memory allocation mmap failed errno 12 on the roboRIO, Source Nodes

### Community 43 - "Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration"
Cohesion: 0.40
Nodes (4): Answer, Outcome, Q: The hood encoder seems to move through zero, and has a discontinuity in the graph when absolute rotations is viewed on advantage scope and the hood is moved manually from its lowest to its highest position. help me figure out how to obtain values to modify the CanCoder configuration, Source Nodes

### Community 44 - "gradlew"
Cohesion: 0.83
Nodes (3): gradlew script, die(), warn()

## Knowledge Gaps
- **81 isolated node(s):** `REAL`, `SIM`, `OIConstants`, `NeoMotorConstants`, `GyroConstants` (+76 more)
  These have ≤1 connection - possible missing edges or undocumented components.
- **6 thin communities (<3 nodes) omitted from report** — run `graphify query` to explore isolated nodes.

## Work-memory lessons

**Preferred sources** — corroborated by past sessions; start here.
- `RobotContainer` (6× useful, score=4.103565189) _(code changed — re-verify)_
- `Constants` (6× useful, score=4.078928059)
- `Turret` (6× useful, score=3.709970173)
- `Vision` (4× useful, score=2.750694385)
- `RobotStateMachine` (4× useful, score=2.642055059)
- `PhotonVisionIO` (3× useful, score=2.260539959)
- `TunerConstants` (3× useful, score=1.867769328)
- `Flywheel` (3× useful, score=1.758208413) _(code changed — re-verify)_
- `AlignTurretToHub` (3× useful, score=1.64284461)
- `UpToSpeedHopperShoot` (2× useful, score=1.012592047)

## Suggested Questions
_Questions this graph is uniquely positioned to answer:_

- **Why does `LimelightHelpers` connect `LimelightHelpers` to `CommandSwerveDrivetrain`, `Pose2d`, `LimelightIO`, `Pose3d`, `LimelightTarget_Barcode`, `RobotContainer.java`, `PoseEstimate`, `.getLimelightNTDoubleArray`, `.getLimelightNTTableEntry`?**
  _High betweenness centrality (0.216) - this node is a cross-community bridge._
- **Why does `RobotStateMachine` connect `RobotStateMachine` to `Command`, `Turret`, `CommandSwerveDrivetrain`, `.getState`, `RunHopper`, `Robot`, `RobotContainer.java`, `Vision`, `.periodic`, `Flywheel`, `.periodic`?**
  _High betweenness centrality (0.122) - this node is a cross-community bridge._
- **Why does `RobotContainer` connect `RobotContainer.java` to `Command`, `TunerConstants`, `Turret`, `PhotonVisionIO`, `CommandSwerveDrivetrain`, `Intake`, `Robot`, `Vision`, `LedCANdle`, `Telemetry`, `Flywheel`, `RobotStateMachine`, `.periodic`?**
  _High betweenness centrality (0.113) - this node is a cross-community bridge._
- **What connects `REAL`, `SIM`, `OIConstants` to the rest of the system?**
  _81 weakly-connected nodes found - possible documentation gaps or missing edges._
- **Should `PhotonVisionSimIO` be split into smaller, more focused modules?**
  _Cohesion score 0.06836055656382335 - nodes in this community are weakly interconnected._
- **Should `TunerConstants` be split into smaller, more focused modules?**
  _Cohesion score 0.06755260243632337 - nodes in this community are weakly interconnected._
- **Should `Turret` be split into smaller, more focused modules?**
  _Cohesion score 0.06271186440677966 - nodes in this community are weakly interconnected._
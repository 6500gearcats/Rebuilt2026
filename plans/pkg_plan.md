# Rebuilt2026 — Package Reorganization Plan

Consolidates split/misplaced packages so the folder structure matches the code's
actual architecture. No logic changes — package declarations and import statements only.

Track execution in `PKG_PROGRESS.md`.

**Branch:** `leto`

---

## Problems Being Fixed

| Problem | Detail |
|---------|--------|
| Two packages named the same thing | `util/` and `utility/` both mean "helper code" |
| Top-level `vision/` shadows `subsystems/vision/` | `vision/localization/` exists for one file, confusingly separate from the vision subsystem |
| `Telemetry.java` in root package | Tightly coupled to `CommandSwerveDrivetrain`; belongs with the drivetrain in `subsystems/` |
| `utility/` contents are subsystem-specific | `ShooterValuesSenable` is shooter code; `SysIDUtil` is drivetrain code |

---

## Target Structure

```
frc/robot/
├── aiming/              (unchanged)
├── commands/            (unchanged)
├── generated/           (unchanged — auto-generated)
├── subsystems/
│   ├── CommandSwerveDrivetrain.java   (unchanged)
│   ├── SysIDUtil.java                 ← moved from utility/
│   ├── Telemetry.java                 ← moved from root
│   ├── hopper/          (unchanged)
│   ├── intake/          (unchanged)
│   ├── shooter/
│   │   ├── Shooter.java               (unchanged)
│   │   ├── ShooterConstants.java      (unchanged)
│   │   ├── ShooterIO.java             (unchanged)
│   │   ├── ShooterIOHardware.java     (unchanged)
│   │   ├── ShooterIOSim.java          (unchanged)
│   │   └── ShooterValuesSenable.java  ← moved from utility/
│   ├── turret/          (unchanged)
│   └── vision/
│       ├── Vision.java                (unchanged)
│       ├── VisionEstimate.java        (unchanged)
│       ├── VisionIO.java              (unchanged)
│       ├── LocalizationConstants.java ← moved from vision/localization/
│       └── photonvision/  (unchanged)
├── superstructure/      (unchanged)
└── util/
    ├── OnboardLogger.java             (unchanged)
    └── StatusSignalUtil.java          (unchanged)
    ← utility/ dissolved; its files moved above
```

Packages `utility/` and `vision/localization/` are deleted (empty after moves).

---

## P-1 — Move `Telemetry.java` to `subsystems/`

**From:** `frc.robot.Telemetry`  
**To:** `frc.robot.subsystems.Telemetry`

**Files to update:**
- `Telemetry.java` — update `package` declaration
- `RobotContainer.java` — imports `frc.robot.Telemetry`

---

## P-2 — Move `SysIDUtil.java` to `subsystems/`

**From:** `frc.robot.utility.SysIDUtil`  
**To:** `frc.robot.subsystems.SysIDUtil`

**Files to update:**
- `SysIDUtil.java` — update `package` declaration
- Any file importing `frc.robot.utility.SysIDUtil` (check `RobotContainer.java`)

---

## P-3 — Move `ShooterValuesSenable.java` to `subsystems/shooter/`

**From:** `frc.robot.utility.ShooterValuesSenable`  
**To:** `frc.robot.subsystems.shooter.ShooterValuesSenable`

**Files to update:**
- `ShooterValuesSenable.java` — update `package` declaration
- Any file importing `frc.robot.utility.ShooterValuesSenable`

---

## P-4 — Move `LocalizationConstants.java` to `subsystems/vision/`

**From:** `frc.robot.vision.localization.LocalizationConstants`  
**To:** `frc.robot.subsystems.vision.LocalizationConstants`

**Files to update:**
- `LocalizationConstants.java` — update `package` declaration
- Any file importing `frc.robot.vision.localization.LocalizationConstants`

---

## P-5 — Delete empty packages

After P-1 through P-4:
- Delete empty `utility/` directory
- Delete empty `vision/localization/` directory
- Delete empty `vision/` directory

---

## P-6 — Move drivetrain files into `subsystems/drivetrain/`

`CommandSwerveDrivetrain`, `Telemetry`, and `SysIDUtil` currently sit loose in
`subsystems/`. Every other subsystem has its own subfolder. Moving these three files
makes the pattern uniform: `subsystems/` contains only subfolders, one per mechanism.

**From → To:**
- `frc.robot.subsystems.CommandSwerveDrivetrain` → `frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain`
- `frc.robot.subsystems.Telemetry` → `frc.robot.subsystems.drivetrain.Telemetry`
- `frc.robot.subsystems.SysIDUtil` → `frc.robot.subsystems.drivetrain.SysIDUtil`

**Files to update:**
- `CommandSwerveDrivetrain.java`, `Telemetry.java`, `SysIDUtil.java` — package declarations
- `RobotContainer.java` — imports `CommandSwerveDrivetrain` and `SysIDUtil`
- `RobotStateMachine.java` — imports `CommandSwerveDrivetrain`
- `generated/TunerConstants.java` — imports `CommandSwerveDrivetrain` (auto-generated; update carefully)
- `generated/TunerConstants2.java` — imports `CommandSwerveDrivetrain` (auto-generated; update carefully)

---

## Commit Strategy

One commit per phase — each is a pure mechanical refactor with no logic changes.

| Commit tag | Content |
|------------|---------|
| `refactor-pkg-consolidation` | All four file moves + import updates + empty dir removal |

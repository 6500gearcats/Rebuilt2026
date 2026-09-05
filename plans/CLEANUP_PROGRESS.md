# Rebuilt2026 — Cleanup Progress

Tracks execution of post-refactor dead code removal. See `cleanup.md` for full rationale on each stage.

**Branch:** `leto`  
**Last updated:** 2026-09-05

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started / deferred

---

## C-1 — RobotStateMachine: Dead `targetPose` State

`updateTargetPose()` was deleted in Stage 5. The field and publisher were never updated again.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| C1-1 | Grep confirms zero external callers of `getTargetPose()` / `isFacingHub()` | ✅ | — |
| C1-2 | Remove `private Pose2d targetPose` field | ✅ | cleanup-c1-c4 |
| C1-3 | Remove `targetPosePublisher` NT4 publisher (was never `.set()`) | ✅ | cleanup-c1-c4 |
| C1-4 | Remove `getTargetPose()` method | ✅ | cleanup-c1-c4 |
| C1-5 | Fix `isFacingHub()` — was reading dead `targetPose`; now reads live `HubPose` | ✅ | cleanup-c1-c4 |

**Result:** `isFacingHub()` now returns a meaningful value. NT4 no longer publishes a bogus `StateMachine/TargetPose` at (0, 0).

---

## C-2 — RobotStateMachine: Dead `getBestPoseTarget()` Method

Private method, zero callers anywhere in the codebase.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| C2-1 | Grep confirms zero callers | ✅ | — |
| C2-2 | Delete `getBestPoseTarget()` method body | ✅ | cleanup-c1-c4 |
| C2-3 | Remove now-unused `import java.util.Optional` | ✅ | cleanup-c1-c4 |

---

## C-3 — RobotContainer: Unused Swerve Requests

`brake` and `point` were CTRE template placeholders never bound to any trigger.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| C3-1 | Grep confirms `brake` and `point` have no callers | ✅ | — |
| C3-2 | Remove `SwerveRequest.SwerveDriveBrake brake` field | ✅ | cleanup-c1-c4 |
| C3-3 | Remove `SwerveRequest.PointWheelsAt point` field | ✅ | cleanup-c1-c4 |

**Note:** `SwerveRequest` import retained — still used for `FieldCentric` and `Idle`.

---

## C-4 — RobotContainer: Unused LedCANdle Instance

`m_candle` was instantiated but never passed to any command or subsystem. LED logic lives in `RobotStateMachine` color fields.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| C4-1 | Confirm `LedCANdle` not instantiated in `RobotStateMachine` (no CAN ID conflict) | ✅ | — |
| C4-2 | Remove `private LedCANdle m_candle` field | ✅ | cleanup-c1-c4 |
| C4-3 | Remove `import frc.robot.subsystems.LedCANdle` | ✅ | cleanup-c1-c4 |

---

## C-5 — SysID Stubs (Deferred to Stage 8)

`m_turretSysID` and `m_flywheelSysID` are intentional placeholders for hardware characterization.  
No code changes until the robot is assembled and Stage 8 begins.

| Task | Description | Status |
|------|-------------|--------|
| C5-1 | Implement turret SysID routine (Stage 8) | ⬜ |
| C5-2 | Bind turret SysID to driver buttons (Stage 8) | ⬜ |
| C5-3 | Implement flywheel SysID routine (Stage 8) | ⬜ |
| C5-4 | Run, export, fit constants, update `TurretConstants` / `ShooterConstants` (Stage 8) | ⬜ |

---

## Summary

| Stage | Files Changed | Status |
|-------|--------------|--------|
| C-1 | `RobotStateMachine.java` | ✅ |
| C-2 | `RobotStateMachine.java` | ✅ |
| C-3 | `RobotContainer.java` | ✅ |
| C-4 | `RobotContainer.java` | ✅ |
| C-5 | — (Stage 8) | ⬜ |

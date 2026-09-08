# Rebuilt2026 — Simulation Progress

Tracks execution of simulation improvement work. See `sim_plan.md` for full rationale.

**Branch:** `leto`  
**Last updated:** 2026-09-08

---

## Legend
- ✅ Done (committed)
- 🔄 In progress
- ⬜ Not started

---

## S-1 — Remove Limelight Dead Code

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S1-1 | Grep confirms zero external callers of `LimelightIO` | ✅ | — |
| S1-2 | Delete `limelight/` package directory | ✅ | sim-s1 |
| S1-3 | Strip `VisionEstimate.java` to PhotonVision-only | ✅ | sim-s1 |
| S1-4 | Compile verify — BUILD SUCCESSFUL | ✅ | sim-s1 |

---

## S-2 — Fix PhotonVision API Deprecations

6 `[removal]` warnings in `PhotonVisionIO.java` and `PhotonVisionSimIO.java`.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S2-1 | Map all 6 deprecation sites across both files | ⬜ | — |
| S2-2 | Update `PhotonPoseEstimator` constructor calls (2 sites) | ⬜ | — |
| S2-3 | Replace `getLatestResult()` with `getAllUnreadResults()` loop | ⬜ | — |
| S2-4 | Replace deprecated `estimator.update(result)` calls | ⬜ | — |
| S2-5 | Compile — confirm zero warnings | ⬜ | — |

---

## S-3 — ShooterIOSim: Flywheel Ramp Dynamics

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S3-1 | Add `targetVelocity` field; decouple `setVelocity()` from `updateInputs()` | ⬜ | — |
| S3-2 | Add exponential filter (0.92/0.08) for ramp-up and ramp-down | ⬜ | — |
| S3-3 | Verify `isUpToSpeed()` threshold still works with the lag | ⬜ | — |
| S3-4 | Run auto in sim — confirm shot waits for flywheel spin-up | ⬜ | — |

---

## S-4 — Hopper: CTRE SimState Feedback

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S4-1 | Add `getSimState().setRotorVelocity()` calls inside `isSimulation()` guard | ⬜ | — |
| S4-2 | Define `kFreeSpeedRPS` constant (~100 RPS for Falcon 500) | ⬜ | — |
| S4-3 | Confirm `Hopper/IndexerVelocityRPS` non-zero in sim | ⬜ | — |

---

## S-5 — Intake: CTRE SimState Feedback

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S5-1 | Read `Intake.java` — identify motor fields and control modes | ⬜ | — |
| S5-2 | Add simState seeding for roller motor (velocity) | ⬜ | — |
| S5-3 | Add simState seeding for deploy motor (velocity + position integration) | ⬜ | — |
| S5-4 | Confirm Intake telemetry shows realistic values in sim | ⬜ | — |

---

## S-6 — Auto Path Validation

| Task | Description | Status |
|------|-------------|--------|
| S6-1 | Run all named auto paths in sim once; note crashes or bad deviations | ⬜ |
| S6-2 | Verify `ShootFuel3s`–`ShootFuel10s` timeouts end command sequences | ⬜ |
| S6-3 | Confirm `AlignTurret` named command moves turret in sim | ⬜ |
| S6-4 | Document path-specific issues found | ⬜ |

---

## Summary

| Stage | Files | Status |
|-------|-------|--------|
| S-1 | `limelight/` (deleted), `VisionEstimate.java` | ✅ |
| S-2 | `PhotonVisionIO.java`, `PhotonVisionSimIO.java` | ⬜ |
| S-3 | `ShooterIOSim.java` | ⬜ |
| S-4 | `Hopper.java` | ⬜ |
| S-5 | `Intake.java` | ⬜ |
| S-6 | — (validation run) | ⬜ |

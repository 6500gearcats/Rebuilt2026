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

Started with 7 `[removal]` warnings. Fixed 5; 2 remain due to PhotonVision RC API state.

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S2-1 | Map all deprecation sites across both files | ✅ | — |
| S2-2 | Replace deprecated 3-arg constructor with 2-arg (strategy removed from ctor) | ✅ | sim-s2 |
| S2-3 | Replace `getLatestResult()` with `getAllUnreadResults()` + cache | ✅ | sim-s2 |
| S2-4 | Replace `estimator.update(result)` with 4-arg overload | ✅ | sim-s2 |
| S2-5 | Zero warnings | ⬜ blocked — in v2026.1.1-rc-3 ALL update() overloads are deprecated; no non-deprecated replacement exists yet in this RC. Revisit when PhotonVision publishes a stable v2026 release. |

**Net result:** 7 → 2 warnings. Remaining 2 are `update()` calls — both files, one site each.

---

## S-3 — ShooterIOSim: Flywheel Ramp Dynamics

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S3-1 | Add `targetVelocity` field; decouple `setVelocity()` from `updateInputs()` | ✅ | sim-s3-s5 |
| S3-2 | Add exponential filter (alpha=0.94, ~1 s to 95%) for ramp-up and ramp-down | ✅ | sim-s3-s5 |
| S3-3 | `Shooter.tracked()` reads `inputs.shooter1Velocity` — lag now blocks it until speed reached | ✅ | sim-s3-s5 |
| S3-4 | Run auto in sim — confirm shot waits for flywheel spin-up | ⬜ deferred to S-6 |

---

## S-4 — Hopper: CTRE SimState Feedback

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S4-1 | Add `getSimState().setRotorVelocity()` inside `RobotBase.isSimulation()` guard | ✅ | sim-s3-s5 |
| S4-2 | `kFreeSpeedRPS = 100.0` (Falcon 500 ~6380 RPM) defined inline in sim block | ✅ | sim-s3-s5 |
| S4-3 | Confirm `Hopper/IndexerVelocityRPS` non-zero in sim | ⬜ deferred to S-6 |

---

## S-5 — Intake: CTRE SimState Feedback

| Task | Description | Status | Commit |
|------|-------------|--------|--------|
| S5-1 | `Intake.java` — two TalonFX motors: roller (duty cycle) and deploy (duty cycle + position) | ✅ | sim-s3-s5 |
| S5-2 | Roller: `setRotorVelocity(get() * kFreeSpeedRPS)` | ✅ | sim-s3-s5 |
| S5-3 | Deploy: `setRotorVelocity()` + `addRotorPosition(vel * 0.02)` per loop | ✅ | sim-s3-s5 |
| S5-4 | Confirm Intake telemetry shows realistic values in sim | ⬜ deferred to S-6 |

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
| S-2 | `PhotonVisionIO.java`, `PhotonVisionSimIO.java` | ⚠️ partial (2 warnings remain, RC blocker) |
| S-3 | `ShooterIOSim.java` | ✅ |
| S-4 | `Hopper.java` | ✅ |
| S-5 | `Intake.java` | ✅ |
| S-6 | — (validation run) | ⬜ |

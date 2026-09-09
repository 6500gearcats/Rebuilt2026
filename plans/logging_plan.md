# Rebuilt2026 — Motor Electrical Logging Plan

Goal: durable, detailed voltage/current/energy logging for every motor on the robot (16
TalonFX total: 8 drivetrain, 3 shooter, 1 turret, 2 intake, 2 hopper), written to the
`.wpilog` file so it survives past a session and is visible in AdvantageScope.

Track execution in `LOGGING_PROGRESS.md`.

**Branch:** `leto`
**Trigger:** User asked whether detailed voltage/current/energy logging exists for all
motors. Audit found it does not — see below.

---

## Audit Summary (verified against code, not assumed)

| Subsystem | Motors | Voltage | Supply Current | Stator Current | Energy | Logged where |
|---|---|---|---|---|---|---|
| Drivetrain | 4 drive + 4 steer TalonFX | ❌ | ❌ | ❌ | ❌ | Nowhere — only kinematic state (`Telemetry.java`) |
| Shooter | shooter1, shooter2, hood TalonFX | ✅ read | ✅ read | ✅ read | ❌ | **Nowhere** — read into `ShooterIOInputs`, used only for control math |
| Turret | 1 TalonFX | ✅ read | ✅ read | ✅ read | ❌ | **Nowhere** — same pattern as Shooter |
| Intake | roller, deploy TalonFX | ❌ | ❌ | ✅ | ❌ | `SmartDashboard.putNumber` → `DataLogManager` |
| Hopper | indexer, kicker TalonFX | ❌ | ❌ | ✅ | ❌ | `SmartDashboard.putNumber` → `DataLogManager` |

**Root blocker, found during this audit:** `OnboardLogger.logAll()` — the method that
actually writes any registered signal to the `.wpilog` file — is **never called anywhere**
in the codebase (verified by repo-wide grep). `Shooter` and `Turret` already register a few
values through `OnboardLogger` (Hood Reference, Velocity Reference, Ready, Tracking, etc.)
but because `logAll()` has no call site, **none of that has ever been written**. This is not
new breakage from this plan — it's a pre-existing silent failure that this plan must fix
first, or every other change below is equally silent.

`CTRE SignalLogger.start()` is also not called anywhere — the CAN-level auto-capture
mechanism an earlier session's notes assumed was active does not exist in this codebase.
See Design Decision below for why this plan does not add it.

---

## Design Decision: standardize on `OnboardLogger`, not `SignalLogger`

`OnboardLogger` (`util/OnboardLogger.java`) writes typed signals directly to the WPILib
`.wpilog` `DataLog`, with unit metadata AdvantageScope reads natively, and without touching
NT4 bandwidth. It's already the established pattern for durable telemetry in this codebase
(see D2-2 in `doc_plan.md`). This plan:

- Fixes L-0 (the dead `logAll()` call) first — everything else depends on it.
- Adds one new capability to `OnboardLogger`: `registerEnergy()`, integrating
  voltage × current × dt into a running Joule total. Nothing in the codebase computes
  energy today; this is new, not a wiring fix.
- Routes Shooter/Turret's already-read voltage/current/temperature into `OnboardLogger`
  (data already in hand, just never published).
- Adds the missing voltage + supply-current reads to Intake/Hopper, then migrates their
  current logging off ad hoc `SmartDashboard.putNumber` onto `OnboardLogger`, for one
  consistent path across every motor.
- Verified via CTRE's Phoenix 6 sources (`SwerveDrivetrain.java`, `SwerveModule.java`,
  v26.1.0) that `drivetrain.getModules()[i].getDriveMotor()` / `.getSteerMotor()` return the
  real `TalonFX` instances — this makes drivetrain motor logging possible without touching
  generated code.
- Leaves `SignalLogger.start()` off. It would duplicate this data into a second file
  (`.hoot`) at the CAN level; `OnboardLogger` gets everything this plan targets into the
  existing `.wpilog` without a second pipeline. Revisit only if a future need (CAN bus
  diagnostics below the StatusSignal level) specifically requires it.

---

## L-0 — Fix `OnboardLogger.logAll()`: never called

**File:** `src/main/java/frc/robot/Robot.java`

Add `OnboardLogger.logAll();` to `robotPeriodic()`, unconditionally (every loop, not
rate-limited — these are per-loop electrical signals, not slow-moving health metrics like
the existing 10 Hz `Robot/BatteryVoltageV` block).

**This must land and be verified before L-2 through L-6** — otherwise every subsequent
registration is silently inert, exactly as Shooter/Turret's existing ones have been.

---

## L-1 — `OnboardLogger`: add energy accumulation

**File:** `src/main/java/frc/robot/util/OnboardLogger.java`

Add `registerEnergy(String name, Supplier<Voltage> voltage, Supplier<Current> current)`:
- Maintains a running Joule total and a last-timestamp, both mutable across `log()` calls
  (a small holder, since lambdas need effectively-final captures)
- Each `log()` call: `dt = now − lastTimestamp` via `Timer.getFPGATimestamp()`;
  `energyJoules += voltage.get().in(Volts) * current.get().in(Amps) * dt`
- Writes the running total as a `DoubleLogEntry` at `<namespace>/<name>EnergyJ`
- Javadoc must be explicit: this is Joules accumulated since robot code start (process
  lifetime), not since last enable — matches how `OnboardLogger`'s other registrations
  already behave (no reset-on-enable mechanism exists in this class)

---

## L-2 — Shooter: log existing voltage/current/temperature + energy

**File:** `src/main/java/frc/robot/subsystems/shooter/Shooter.java`

`ShooterIOInputs` already carries `shooter1/shooter2/hoodVoltage`, `SupplyCurrent`,
`StatorCurrent`, `Temperature` — populated every loop by `ShooterIOHardware.updateInputs()`,
just never logged. Add to the existing `OnboardLogger log = new OnboardLogger("Shooter")`
block in the constructor:
- `Shooter1/Voltage`, `Shooter1/SupplyCurrentA`, `Shooter1/StatorCurrentA`, `Shooter1/TempC`
- Same three for `Shooter2` and `Hood`
- `registerEnergy` for each of the 3 motors

No new hardware reads needed — suppliers just read the already-populated `inputs` struct.

---

## L-3 — Turret: same treatment

**File:** `src/main/java/frc/robot/subsystems/turret/Turret.java`

`TurretIOInputs` already carries `voltage`, `supplyCurrent`, `statorCurrent`,
`torqueCurrent`, `temperature`. Add to the existing `OnboardLogger log = new
OnboardLogger("Turret")` block:
- `Voltage`, `SupplyCurrentA`, `StatorCurrentA`, `TorqueCurrentA`, `TempC`
- `registerEnergy`

---

## L-4 — Intake: add missing voltage + supply current, migrate to `OnboardLogger`

**File:** `src/main/java/frc/robot/subsystems/intake/Intake.java`

Currently only `getStatorCurrent()` is read for either motor. Add `getMotorVoltage()`
(matches the API `ShooterIOHardware`/`TurretIOHardware` already use) and
`getSupplyCurrent()` for both roller and deploy motors. Replace the ad hoc
`SmartDashboard.putNumber` current calls with an `OnboardLogger("Intake")` registering:
- `Roller/Voltage`, `Roller/SupplyCurrentA`, `Roller/StatorCurrentA`, `registerEnergy`
- `Deploy/Voltage`, `Deploy/SupplyCurrentA`, `Deploy/StatorCurrentA`, `registerEnergy`

Leave the existing position/velocity `SmartDashboard.putNumber` calls alone — this plan is
scoped to electrical telemetry.

---

## L-5 — Hopper: same treatment

**File:** `src/main/java/frc/robot/subsystems/hopper/Hopper.java`

Same pattern as L-4 for indexer and kicker: add voltage + supply current reads, migrate
current logging to `OnboardLogger("Hopper")`, add `registerEnergy` for both.

---

## L-6 — Drivetrain: expose and log all 8 module motors (largest gap)

**File:** `src/main/java/frc/robot/subsystems/drivetrain/CommandSwerveDrivetrain.java`

Verified API (Phoenix 6 `wpiapi-java` 26.1.0 sources,
`com/ctre/phoenix6/swerve/SwerveDrivetrain.java` / `SwerveModule.java`):
`getModules()[i].getDriveMotor()` and `.getSteerMotor()` both return the real `TalonFX`.

Tasks:
- Register bulk status-signal refresh for all 8 motors' voltage/current/temperature via
  `StatusSignalUtil.registerRioSignals()`, matching the pattern in
  `ShooterIOHardware`/`TurretIOHardware`. Do this once, in the constructor, alongside the
  existing sim-thread setup.
- Add an `OnboardLogger("Drivetrain")` registering, per module (0=FL, 1=FR, 2=BL, 3=BR,
  per the class's own module-indexing convention):
  - `Module{i}/DriveVoltage`, `.../DriveSupplyCurrentA`, `.../DriveStatorCurrentA`, `.../DriveTempC`
  - `Module{i}/SteerVoltage`, `.../SteerSupplyCurrentA`, `.../SteerStatorCurrentA`, `.../SteerTempC`
  - `registerEnergy` for all 8 motors
- This closes the single largest gap (8 of 14 motors, currently zero electrical telemetry)
  and needs the most new code, since there's no existing IO-layer struct to piggyback on.

**Verify during implementation:** confirm reading `StatusSignal`s from these `TalonFX`
instances doesn't reintroduce loop-timing issues — `StatusSignal` reads are independent of
CTRE's internal odometry-thread writes, but this session already found one real lock-
contention issue (`getState()` vs. the sim notifier) so re-check rather than assume.

---

## Commit Strategy

| Commit | Files |
|---|---|
| `logging-l0-l1-fix-logall-add-energy` | `Robot.java`, `util/OnboardLogger.java` |
| `logging-l2-l3-shooter-turret` | `subsystems/shooter/Shooter.java`, `subsystems/turret/Turret.java` |
| `logging-l4-l5-intake-hopper` | `subsystems/intake/Intake.java`, `subsystems/hopper/Hopper.java` |
| `logging-l6-drivetrain` | `subsystems/drivetrain/CommandSwerveDrivetrain.java` |

Compile + commit + push after each stage.

---

## Out of Scope (noted, not done here)

- Re-enabling CTRE `SignalLogger.start()` — see Design Decision above.
- An aggregate whole-robot power/energy dashboard tile — a fast follow once all
  `registerEnergy` totals exist (just sum them), but not requested and not built here.
- CANdle/LED controller and PathPlanner-managed devices — not motors, out of scope.
- Climber — no motor exists on this robot; nothing to log.

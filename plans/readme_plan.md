# Rebuilt2026 — Root README Rewrite Plan

Extracts the architecture-reference content currently stuck in `plans/README.md` (stale,
Hackbots/SPARK-MAX-era) into a new, accurate root-level `README.md`, and shrinks
`plans/README.md` to a short index of the `plans/` folder. Written and executed in one
sitting per `plans/PLANNING_GUIDE.md`'s own guidance — no long gap between claim and
verification.

Track execution in `README_PROGRESS.md`.

**Branch:** `leto`

---

## Why a rewrite, not a move

`plans/README.md` describes: REV SPARK MAX swerve motors (actual: CTRE TalonFX via
`TunerConstants2`), `Flywheel.java` (deleted Stage 5, replaced by `Shooter`/`ShooterIO`),
`RangeFinder.java` (deleted Stage 5, replaced by the `aiming/` package), a "Partial" Climber
(fully deleted Stage 3), Limelight cameras (deprecated Stage 1, PhotonVision-only now),
commands like `ShootingSequenceUTS`/`AlignTurretToHub`/`MoveTurret` (all deleted Stage 5,
replaced by `AimPrep`/`ShootWhenReady`), a `NamedCommands` table with removed climb entries
and missing real ones, and a CAN ID table that predates today's `logging_plan.md`
reassignments. Nearly every section needs re-deriving from current code, not copying.

## Ground rule (per `PLANNING_GUIDE.md`)

Every fact in the new README must be verified against current source in this session, not
carried over from memory of the old README or from earlier context in this conversation
that might itself be stale. Cite what was checked in `README_PROGRESS.md`.

---

## RM-1 — Research: gather ground truth

Read/grep what the new README will describe, in full, before writing anything:
- `RobotContainer.java` (full) — subsystems, bindings, NamedCommands
- `RobotStateMachine.java` (full) — states, zones, singleton responsibilities
- `commands/*.java` (full file list via Glob) — current command roster
- CAN ID sources: `TunerConstants2.java` (swerve), `Constants.MotorConstants` (intake/hopper),
  `ShooterConstants`/`TurretConstants` (post `logging_plan.md`)
- Vendor library actually-used check: grep for SPARK MAX / REVLib usage — is REVLib dead
  weight now that swerve is TalonFX-based?
- `LedCANdle` — does the class/live instance still exist anywhere after `cleanup.md` C-4
  removed `RobotContainer`'s copy?
- DIO pins, network ports, PathPlanner auto file list

## RM-2 — Write new root `README.md`

Sections, each verified against RM-1's findings, not the old file's prose:
- What this robot does (high-level, low-risk to get wrong — light touch)
- Project structure tree (current package layout, post `pkg_plan.md`)
- WPILib primer for new developers (generic WPILib concepts — carry over from old README
  largely as-is, this part wasn't wrong)
- Subsystems (drivetrain, turret, shooter, hopper, intake, vision, LED if it still exists)
- Commands (current roster only)
- Robot state machine
- Vision & pose estimation
- Operator interface & controls
- Autonomous (PathPlanner) + accurate NamedCommands table
- Hardware reference (CAN IDs, DIO, network ports) — current, including today's
  `logging_plan.md` reassignments
- Vendor libraries — accurate to what's actually linked and used
- Building & deploying (generic — carry over largely as-is)
- Tuning & telemetry — update to describe `OnboardLogger`/`.wpilog`-based telemetry, not the
  old flat `SmartDashboard` key table

## RM-3 — Shrink `plans/README.md` to a folder index

Replace the architecture content with: a pointer to the new root `README.md`, a pointer to
`PLANNING_GUIDE.md`, and a one-line-per-file index of everything in `plans/`.

## RM-4 — Final check

Since this is markdown-only, no compile step applies. Instead: re-grep a handful of the new
README's specific claims (a CAN ID, a command name, a file path) against source one more
time before the final commit, as a last sanity pass distinct from RM-1's initial research.

---

## Commit Strategy

| Commit | Content |
|---|---|
| `readme-rm1-rm2` | New root `README.md` |
| `readme-rm3` | Shrunk `plans/README.md` |

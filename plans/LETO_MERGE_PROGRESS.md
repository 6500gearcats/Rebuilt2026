# Leto Merge — Progress Tracker

Companion to [`leto_merge_plan.md`](leto_merge_plan.md). Nothing in this tracker has been started — this file was created alongside the plan, on `leto_main`, with no source changes.

Update this file in the **same commit** as any work it describes, per `leto`'s `plans/PLANNING_GUIDE.md` convention (to be adopted formally once Bucket A lands, §Stage 1). Cite concrete file/method names and commit hashes, never vague prose.

---

## Snapshot at plan creation (2026-09-16)

| Ref | Commit |
|---|---|
| Merge base | `67ee7a5` |
| `leto` tip (target architecture) | `22ba3b1` |
| `leto_main` tip (current branch) | `5372905` |

No commits have been made on any integration branch yet — `leto-integration` (§5, Stage 0) does not exist.

---

## Stage checklist

| Stage | Description | Status | Notes / commit refs |
|---|---|---|---|
| 0 | Cut `leto-integration` from `leto`'s tip (`22ba3b1`); confirm clean build in isolation | Not started | |
| 1 | Land Bucket B as inert copies (`HomeIntake.java`, `RunHopperBack.java`, Choreo/PathPlanner assets) | Not started | |
| 2 | Resolve Bucket C file-by-file (decision log, D-1 style) | Not started | Blocked in part on Q1–Q4 (see below) |
| 3 | Reconcile Bucket D file-by-file, easiest → hardest | Not started | Order fixed in plan §5, Stage 3 |
| 4 | CAN ID / `TunerConstants` / `TunerConstants2` audit | Not started | Blocked on Q1 |
| 5 | Auto/path named-command audit (17+ files identified in plan §3.4.5) | Not started | Depends on Stage 3's `RobotContainer.java` reconciliation |
| 6 | Build & full test-suite verification + sim smoke test | Not started | |
| 7 | Real-robot bring-up checklist (turret zeroing, PID slots, ductTapeCorrection, intake/hopper tuning, RangeFinder port) | Not started | |
| 8 | Finalize — merge/fast-forward into `leto_main` | Not started | Blocked on Q5 |

---

## Decision log

*(Empty. Populate one entry per row of the plan's Bucket C/D tables as each decision is made, in the style of `leto`'s `review_plan.md` D-1..D-6 log: decision, who made it, why, and the commit that applied it.)*

| ID | File / topic | Decision | Commit |
|---|---|---|---|
| — | — | — | — |

---

## Open questions (blocking) — mirrors plan §6

| # | Question | Answered? | Answer / date |
|---|---|---|---|
| Q1 | Does the robot still have the second Pigeon2/CANivore drivetrain (`TunerConstants2.java`)? | No | |
| Q2 | Is a Limelight camera still mounted? | No | |
| Q3 | Is there a climber on the robot? | No | |
| Q4 | Are the CANdle LEDs still installed and wired? | No | |
| Q5 | Fast-forward `leto_main` vs. explicit merge commit at Stage 8? | No | |
| Q6 | Commit generated `docs/` to source control, or regenerate + `.gitignore`? | No | |

---

## Findings during execution

*(Empty — populate as work proceeds. Record anything that invalidates or refines a claim in `leto_merge_plan.md`, and amend the plan itself in the same commit per the process convention.)*

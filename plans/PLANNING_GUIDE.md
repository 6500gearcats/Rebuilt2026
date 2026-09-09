# How We Write Plans and Track Progress

Process guidance for authoring `plans/*_plan.md` + `plans/*_PROGRESS.md` pairs in this
repo. Written 2026-09-09 after `audit_plan.md` found three tracker/code sync issues across
six existing plan pairs — see `AUDIT_PROGRESS.md` for the full findings this guidance is
based on. Nothing here changes any code; it's about how the *paperwork* stays trustworthy.

---

## The three ways trackers drift from code (all observed in this repo)

### 1. A later stage silently invalidates an earlier stage's claim

`Rebuilt2026_RefactorPlan.md` Stage 2 said "telemetry added to Flywheel/Turret." Stage 5
deleted `Flywheel.java`. Nobody amended Stage 2's line — a separate Appendix recorded the
deletion, but in a different section, so a reader of Stage 2 alone never saw it.

**Do this instead:** when a commit deletes or replaces a file, grep the plan docs for that
filename as part of the *same commit*, and annotate every place it's mentioned. Treat this
as part of the deletion's definition of done, not a follow-up task for later.

### 2. The tracker assumes an execution shape that isn't what actually happened

`PKG_PROGRESS.md` specified a two-phase move (land files in `subsystems/` root, then move
again into `subsystems/drivetrain/`). The actual work did it in one commit. Whoever finished
it checked the boxes whose wording matched that one commit message and left the rest
unchecked — not because the work wasn't done, but because the tracker's granularity no
longer corresponded to what really happened.

**Do this instead:** update the tracker in the *same commit* as the code change. If the real
implementation diverges from the plan's shape, edit the plan's task breakdown too — don't
leave stale sub-tasks unchecked when their intent was actually fulfilled a different way.

### 3. A deliberate reversal happens with no record of the decision

`SIM_PROGRESS.md` recorded a PhotonVision API migration as done. Someone later chose to keep
the deprecated constructor instead. Legitimate call — but nothing recorded *why*, so the
tracker kept asserting something false until an audit caught it.

**Do this instead:** a reversal of a "done" item is worth one dated sentence the moment the
decision is made: what changed, why. Costs nothing at decision time; saves a future audit
from re-litigating it.

---

## What made the other plans hold up

`cleanup.md`, `doc_plan.md`, and `logging_plan.md` all survived the audit clean. Two things
they had in common that the drifted plans didn't:

- **Concrete references, not vague ones.** Commit hashes, specific file/method names —
  never "Multiple" as a file column, never a claim you can't `grep` for.
- **One source of truth per fact.** The master plan has both a top-level status table *and*
  detailed per-stage sections describing the same thing — two places that must be kept in
  sync by hand, and one of them (Stage 6's status) wasn't. Don't duplicate a status
  somewhere else unless you're going to update both in the same edit, every time.

The single lowest-drift pattern observed: **`logging_plan.md` was written and executed in
the same sitting**, compile-checking after every stage. There was no window of time for the
plan and the code to drift apart because there was no gap between writing the claim and
verifying it.

---

## Checklist for new plan/progress pairs

- [ ] Name the pair `plans/<topic>_plan.md` + `plans/<TOPIC>_PROGRESS.md`, matching existing
      convention (see `plans/README.md`'s file list for examples).
- [ ] Reference specific files, methods, and (once committed) commit hashes — not prose
      summaries like "Multiple" or "various fixes."
- [ ] Update the progress file in the *same commit* as the code it describes, not as a
      separate follow-up.
- [ ] If a status is recorded in more than one place (a summary table and a detailed
      section, say), update both together or don't duplicate it — pick one.
- [ ] When a later change deletes, replaces, or reverses something an earlier plan claimed
      as done, go amend that earlier claim in the same commit. Don't rely on a separate
      "file change log" to carry the cross-reference.
- [ ] Prefer writing and executing a plan in one sitting over writing it far ahead of the
      work. If a plan must sit unexecuted for a while, re-verify its claims against current
      code before resuming rather than trusting it as written.
- [ ] Before relying on an old plan's "done" claim to build new work on top of it, spot-check
      it against the actual current file — a two-minute grep is cheaper than inheriting a
      stale assumption.

# Shooting calibration

The shot table starts **empty**. Automatic shooting is blocked until at least two
accepted, measured rows are added to `ShotTable.samples()` and deployed. Manual
calibration works without a table or TOF. Do not copy the old flywheel/TOF tables
into this one: they do not describe shots with the new hood.

## Before shooting

1. Compile and deploy. Run the tests first:
   ```sh
   ./gradlew test -Dorg.gradle.java.home="/Users/mitchquinn/wpilib/2026/jdk"
   ```
2. Identify the hardware in Phoenix Tuner: top flywheel ID 13, bottom ID 14,
   hood motor ID 16, hood CANcoder ID 17. The top flywheel leads; the bottom
   follows the actual leader ID. Verify both wheel directions at a low speed.
   The merged clockwise inversion and aligned follower configuration are retained;
   confirm these match the physical shooter before collecting data.
3. Verify flywheel **mechanism RPS**, not RPM or motor rotor RPS. The existing
   sensor-to-mechanism ratio is 0.6; check that ratio against the hardware. Both
   motors must communicate, and the leader must reach the requested speed.
4. With no balls loaded, confirm the hood's CANcoder and motor feedback agree.
   The table uses the zeroed CANcoder coordinate in rotations, **not launch
   angle**, motor rotor turns, or a fraction of available hood travel.
5. Check the configured lower/upper positions (0.012 and 0.673 rotations),
   offset (zero 0.8496), direction, and discontinuity (0.83). Power-cycle at the
   lower end, middle, and upper end. The absolute position and motor feedback
   should agree within 0.005 rotations at each position. Startup seeds the
   feedback from the actual absolute position; it never assumes the hood is
   against its lower stop. Recovery of failed initialization requires disabling.
6. Command a few in-range hood positions and verify repeatable settling. Check
   software limits before tuning shots. If the retained gains cannot settle,
   tune position control before collecting ballistic data.
7. Confirm the turret's zero and homing. Zero turret yaw fires toward the robot's
   rear. Compare aiming and alignment with the physical shooter direction.

Configuration or feedback errors block feeding. They must be resolved rather than
compensated with a different shot-table value.

## Controls and dashboard

Set inputs through the existing SmartDashboard/NetworkTables dashboard. Use
Driver Station **teleop** for calibration; test mode also permits explicit
calibration, while otherwise preserving characterization command ownership.
Calibration is unavailable in autonomous and resets when disabled.

| Dashboard key | Meaning |
| --- | --- |
| `Calibration/Enabled` | Enable exact manual speed/hood targets; starts false |
| `Calibration/Distance Meters` | Independently measured horizontal turret-base-to-hub-center distance |
| `Calibration/Flywheel RPS` | Exact requested mechanism speed; positive to spin |
| `Calibration/Hood Rotations` | Exact zeroed CANcoder position within the hood limits |
| `Calibration/TOF Seconds` | Median video flight time; use 0 until measured |
| `Calibration/Trial ID` | Unique name per setting/distance/batch, also named in the video |
| `Calibration/Attempts`, `Calibration/Scored` | Completed batch result counts |
| `Calibration/Notes` | Miss direction, arc, ball condition, video file/frames/FPS, batch references |
| `Calibration/Capture Trial` | Set true to record; resets itself false after capture |
| `Calibration/Candidate Java Row` | Copyable candidate after a ready manual feed, positive TOF, and valid counts |
| `Shot/Motion Compensation Enabled` | Enable the measured TOF motion calculation; starts false |
| `Shot/Flywheel Trim RPS` | Explicit automatic speed trim; starts/resets to 0 |
| `Shot/Valid`, `Shot/Status`, `Shot/Can Feed` | Solution validity, block reason, and physical feed readiness |

Requested/actual speed and hood position, readiness, pose distance, and battery
voltage also appear under `Calibration/`. `ShotVelocity` is the table speed before
operator trim; `reqSpeed` is the exact speed actually requested.

Gunner POV up/down changes speed by 1 RPS. During calibration it edits the manual
target; otherwise it edits the logged automatic trim. Gunner Y/A changes the
calibration hood target by +/-0.005 rotations and does nothing outside calibration.
Enter exact values on the dashboard when a larger adjustment is needed.

Hold the usual shoot/aim trigger to aim and feed when ready. The existing manual
shoot button also uses all readiness gates, even though it does not schedule auto
alignment: manually aim first or run the alignment command. Release to stop
feeding. The shooter continues holding/spinning at a valid setpoint, as the old
automatic preparation did. Disable or set manual RPS to 0 to stop the flywheel.

All shooting sequences, including legacy autonomous `ShootFuel*` registrations,
require a valid solution, physical alignment within 1.5 degrees, turret speed
within 0.1 motor RPS, settled flywheel, and settled hood. There is no close-range
exception. Hopper ownership is declared so reverse/other hopper commands cancel
shooting cleanly. Generic `RunHopper`/`StaggerHopper` remain material-handling
commands; do not use them as an alternative firing command during calibration.

Flywheel readiness: both wheels within 2 RPS continuously for 0.08 s. Hood readiness: error
<=0.005 rotations continuously for 0.10 s. Meaningful target changes, accumulated
small changes, sensor errors, or stops reset readiness. The follower remains in
follower mode when the leader stops and starts again.

Calibration and stationary automatic mode require turret-base speed <=0.1 m/s
and robot angular speed <=0.1 rad/s. Automatic shots outside alliance territory
or in the current trench regions are blocked; this table describes ordinary hub
shots only. Passing and trench profiles need their own calibration.

## Efficient collection session

Use a driver/tuner and an observer/video operator. Keep the target height, balls,
feed configuration, and hood coordinate fixed throughout a session.

1. **Measure the distance correctly.** Use horizontal turret-base-to-hub-center
   distance in meters. If measuring from the bumper or near edge of the hub,
   account for those offsets. Compare the measured distance with
   `Calibration/Pose Distance Meters`. Resolve a systematic pose/measurement
   difference before changing shooter settings; do not hide it in the table.
2. **Start in the middle.** Choose a comfortable practice distance with a clear
   view of the arc. Enter a unique trial ID, distance, modest trial RPS, and an
   in-range hood position. Leave TOF at 0. Aim, stop the robot, and wait for both
   readiness indicators. Clear the attempt/scored counts for the new batch.
3. **Screen with three balls.** Adjust one setting at a time. Use 1-RPS speed
   steps and 0.005-rotation hood steps near a promising setting. Record short,
   long, left/right, rim contact, and whether the ball enters while descending.
   If the initial guess is far off, use larger dashboard adjustments, then refine.
4. **Prefer a forgiving descending arc.** Find a combination that tolerates ball
   variation and feed disturbances. Among similarly reliable combinations,
   choose the shorter flight time. Do not switch between incompatible low/high
   arcs at adjacent rows: interpolation between them may not score.
5. **Repeat two ten-ball batches.** Accept a candidate only when each batch scores
   at least 9/10. Keep each batch's trial ID and video reference. The generated
   row is a candidate, not automatic approval of its measured quality.
6. **Measure flight time** for at least three scored shots at those exact settings
   as described below. Record all times and use their median for the row.
7. **Capture the trial.** After the batch, enter results, median TOF, and notes,
   then set `Calibration/Capture Trial` true. A matching trial ID reuses the
   most recent ready feed-start snapshot, preserving its original requested and
   actual settings even if you have disabled since then. A capture without a
   matching feed is diagnostic only and cannot generate a candidate row.
8. **Expand the range.** Tune near and far anchors, then fill approximately 0.5 m
   gaps. Carry forward the nearest successful settings. Add more rows where the
   arc or settings change rapidly; do not run an exhaustive two-dimensional sweep.
9. **Update the Java table.** Copy only accepted candidate rows into
   `ShotTable.samples()`, ordered by increasing distance. Record their trial IDs
   in the provenance ledger below. Deploy the table and test the midpoint of
   every adjacent pair. Apply the same repeated-batch criterion; add a measured
   row where interpolation misses or produces an unsuitable arc.

For long or fast-feeding batches, observe speed recovery throughout the batch.
The gate stops feeding when readiness is lost and resumes when it returns. A new
feed-start event is recorded for each stopped-to-feeding transition, not every
ball. Individual ball exits cannot be inferred from those events.

## Video TOF measurement

Film shooter exit and the hub opening in the same recording, ideally from the
side. Use the recording's **capture FPS**, not the slowed playback frame rate.
Show or announce the trial ID so the clip matches the dashboard/log.

- Release frame: ball has just left contact with the shooter.
- Arrival frame: ball crosses the hub opening plane.
- `TOF seconds = (arrival frame - release frame) / capture FPS`.
- Measure at least three successful balls; use the median. Keep frame indices,
  capture FPS, and spread in the notes. Large variation is a reason to investigate
  ball/feed variation or uncertain frame selection before accepting the row.

Example: 180 frames at 240 capture FPS gives 0.750 s. This is a timing example,
not a calibration value. Hopper-start-to-arrival includes feeding delay and is
not the TOF used for motion compensation.

`ShotFeedStart` records FPGA time, sequence number, trial ID, measured/pose
distance, requested/actual RPS and hood position, readiness, explicit speed trim,
and battery voltage.
`ShotTrial` adds capture time, results, video TOF, notes, and candidate row.
`DataLogManager.log()` saves these entries to the existing `.wpilog` and prints
them to the Driver Station console. Retrieve logs from the robot/USB and inspect
the `messages` entry in WPILib's DataLogTool or a compatible log viewer. Also
retain the annotated video and the ledger; the robot does not automatically
rewrite or persist an accepted Java table.

## How interpolation works

Synthetic illustration only: rows at 2 m and 4 m with `(40 RPS, 0.1 rotations,
0.5 s)` and `(60 RPS, 0.3 rotations, 1.0 s)` give at 2.5 m:

```text
fraction = (2.5 - 2) / (4 - 2) = 0.25
speed    = 40 + 0.25 * (60 - 40) = 45 RPS
hood     = 0.1 + 0.25 * (0.3 - 0.1) = 0.15 rotations
TOF      = 0.5 + 0.25 * (1.0 - 0.5) = 0.625 seconds
```

All settings share one fraction and pair of rows. Endpoints are valid. There is
no extrapolation, polynomial fitting, or hidden angle/trench speed correction.
Duplicate/unsorted distances, nonfinite values, nonpositive speed/TOF, or invalid
hood positions disable the automatic table and report a startup error.

## Moving-shot validation

Keep motion compensation off until stationary anchors and all midpoints pass.
Then set `Shot/Motion Compensation Enabled` true and use zero speed trim.
Validate toward/away travel, lateral travel, and rotation separately at low
speeds before combining them. Repeat batches and inspect the recorded targets,
TOF, raw/effective distance, radial velocity, and physical settings. A nonzero
trim blocks moving shots because the measured TOF no longer describes that speed.

The existing model is retained: `effectiveDistance = distance - radialVelocity *
TOF(effectiveDistance)`. Positive radial velocity moves toward the target. Iterate
at most 20 times and require a <=0.001 m change. Re-query all three settings at
the final effective distance, then aim at `hubPosition - turretVelocity * TOF`.
Turret velocity includes the robot's rotation about the turret offset. Lateral
motion affects aim lead, not the radial range approximation; this is an empirical
approximation to validate on the robot, not an exact ballistic model. Out-of-range
iterations or failed convergence stop feeding.

## Troubleshooting

| Symptom/status | Check |
| --- | --- |
| `Shot table is empty` | Use calibration mode; collect and deploy at least two accepted rows |
| `Invalid calibration inputs` | Positive measured distance/RPS, in-range hood, finite TOF >=0 |
| `Stop robot for manual calibration` | Chassis/turret-base velocity and robot angular velocity |
| `Motion compensation disabled; stop robot` | Stationary tuning first; enable motion only after validation |
| `Passing/trench profile not calibrated` | Move into a supported ordinary hub-shot location |
| No candidate row | Matching feed-start trial ID, settled manual shot, measured TOF, valid counts |
| Hood not ready | Initialization/communication, absolute-vs-feedback coordinate, limits, gains |
| Flywheel not ready | Motor IDs/follower, direction, mechanism ratio, battery, gains |
| Aligned indicator false | Physical rear-facing convention, yaw zero, homing, unreachable yaw target |
| Distance outside calibrated range | Move into the measured interval or collect another accepted anchor |
| Motion solution did not converge | Check TOF rows and velocity; do not feed using the previous solution |

## Accepted-row provenance

Fill this ledger when adding actual rows. No shots have been accepted yet.

| Distance m | Flywheel RPS | Hood rotations | Median TOF s | Batch trial IDs/results | Video/FPS/frames and notes |
| --- | --- | --- | --- | --- | --- |

# SmartDashboard telemetry

The normal dashboard keeps the auto chooser, main `Field` view, controller and
encoder health, mechanism measurements, requested speeds, and game status.
Detailed diagnostics are off at robot startup.

## Enable detailed diagnostics

Set `Telemetry/Debug Enabled` to `true` in SmartDashboard or your NetworkTables
dashboard. Values under `Debug/...` update at most 10 times per second per owner.
The switch is read on the robot thread; drivetrain callbacks use its cached value.
Set it to `false` to stop those updates. Motor commands, shot calculations, pose
fusion, existing structured NetworkTables streams, and disk logging do not depend
on this setting.

Debug field and module widgets are registered on their first enabled update.
Disabling debugging stops their updates; it does not remove their topics or clear
their last displayed values. Per-camera poses show the last accepted robot pose,
and rejection flags describe the last estimate evaluated, not camera freshness.

## Operating display corrections

- `Aligned` resets to false when the alignment command ends.
- `Hood Requested Position (degrees)` reports the position request sent to the
  controller, including automatic shot tracking and manual step commands.
- `Flywheel Speed Within Tolerance` reports the calculated speed readiness after
  the existing dwell time. It does not include the testing override.
- `Flywheel Readiness Override` exposes the existing testing override, which
  remains **true**. Consequently `Up to Speed` remains forced true for compatibility
  with current shooting behavior. Removing that override requires a separate
  behavior review and robot validation.

## Dashboard layout migration

No dashboard layout is tracked in this repository. Update the actual driver and
pit layouts using these mappings. Existing retained/persistent NetworkTables keys
and saved widgets may need to be cleared manually; removing a publisher does not
guarantee that a dashboard forgets its last value.

| Previous key | Replacement or disposition |
| --- | --- |
| `Auto Chooser`, `Field`, `AutoTime` | Retained |
| `reqSpeed`, `actSpeed`, `Up to Speed`, `Flywheel Manual Override` | Retained |
| `Left Motor Speed` | Use `actSpeed` (same feedback, RPS) |
| `Shoot Speed` | Removed; no active reader |
| `Shot Multiplier`, `Rotation Multiplier` | Removed; no longer applied to speed control |
| `flywheel initial speed` | `Debug/Flywheel/Base Speed (RPS)` |
| `rot adder` | `Debug/Flywheel/Rotation Correction (RPS)` (zero when not applied) |
| `flywheel new speed` | `reqSpeed`, or `Debug/Flywheel/Requested Speed (RPS)` |
| `rot new testing`, `rot old testing` | Removed; use the turret angle or debug field heading |
| `switch on or off` | `Turret Limit Switch` (raw input polarity retained) |
| `Turret Position` | `Turret Absolute Position (degrees)` (same angle) |
| `Motor Position` | `Debug/Turret/Feedback Position (rotations)` |
| `Turret Absolute Position (rotations)` | `Debug/Turret/Absolute Position (rotations)` |
| `Robot Rot in Deg` | Use the existing robot pose stream/field view |
| `UnconvPos` | Use debug requested turret angle; rotations were angle / -360 |
| `Controller speed` | `Debug/Turret/Manual Duty Cycle` |
| `errorFromPrev.getX`, `errorFromPrev.getY`, `errorFromPrevRobotRot`, `errroFromPrevRot` | Removed; previous values were never updated |
| `turretAndRobot`, `Dist to Tag`, `turretError`, `tunring_pos_setpoint` | `Debug/Turret/Field Heading (deg)`, `Target Distance (m)`, `Aim Error (deg)`, `Requested Angle (deg)` respectively, under the same prefix |
| `Hood motor Position`, `Hood targetPosition` | `Hood Requested Position (degrees)` for the actual request; `Debug/Hood/Manual Step Target (rotations)` for the manual accumulator |
| `Hood Absolute Position (rotations)` | `Debug/Hood/Absolute Position (rotations)`; degrees remain on the operating display |
| `Hood m_commandedSpeed` | `Debug/Hood/Manual Duty Cycle` |
| `Hood move down blocked`, `Hood move up blocked` | Removed; use the periodically updated `Hood At Lower Limit` / `Hood At Upper Limit` |
| `CamPose<camera>`, `CamPoseRejected<camera>` | `Debug/Vision/CamPose/<camera>`, `Debug/Vision/Pose Rejected/<camera>` |
| `Module 0` through `Module 3` | `Debug/Drive/Module 0` through `Module 3` |

Aiming diagnostics now live under `Debug/Aiming/`, with units in each key: robot
and turret velocities, radial velocity, shot distance, effective shot distance,
flight time, flywheel speed, hood position, and hub/compensated-hub coordinates.
They publish from the state machine periodic telemetry path, using cached shot
results rather than publishing every time a getter recalculates a shot. The
`Debug/Aiming/Targets` field widget shows both target positions.

The inactive `Vision/DataCollection/...` panel and its disconnected trial logger
were removed. Neither its control-update method nor its sample logger was being
called. Restoring trial collection should be a separate test feature.

Game-specific state, switch timing/color, trench status, and duct-tape correction
entries remain available pending next season's behavior review. Turret PID tuning
inputs remain unchanged.

## Verification before the next robot run

Compile and run the Java tests. On the robot or in a configured simulation, check
the chooser, main field, operating measurements, and requested hood position with
debugging off. Then enable debugging, verify the detail views update, and disable
it again. Check alignment cancellation clears `Aligned` and that flywheel readiness
shows both the computed result and the active override. These hardware/dashboard
checks cannot be established by compilation alone.

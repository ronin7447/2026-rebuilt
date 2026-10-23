# Turret target stabilization and feed readiness

These are independent compile-time switches in `Constants.java`. Rebuild and
deploy after editing them. Their dashboard values report the deployed setting;
they are not dashboard overrides.

## Feed readiness

`Constants.ShotMap.FEED_INTERLOCK_ENABLED` defaults to **false**.

- **false:** Normal right-trigger and autonomous `ShootSequence` feeding depend
  only on robot enable. Flywheel speed, hood/turret alignment, solver validity,
  model setpoint bounds and motion readiness do not block these feed motors.
  The existing 50 ms outtake and additional 25 ms spindexer staging remain.
- **true:** Restore the complete readiness snapshot and motion checks, including
  immediate feed interruption and the configured recovery/staging sequence.
- Calibration uses the same switch for measured speed and hood/turret readiness.
  Its valid trial inputs, enabled teleop, enabled shooter and bounded trial
  setpoints remain required in either setting. Calibration has no feed timers.
- The deliberate right-stick override retains its existing role. With readiness
  enabled, it bypasses readiness while retaining its basic command protections.
  With readiness disabled, it skips staging and requires robot enable only.

Robot disable, motor configuration guards, current limits and hardware/software
actuator limits remain effective. `Feed/ReadinessChecksEnabled`, `Feed/Reason`
and `Feed/ReadinessWarnings` distinguish active gating from diagnostic warnings.

## Turret stabilization

`Constants.Turret.TARGET_STABILIZATION_ENABLED` defaults to **true**. Set it to
**false** to restore raw target selection, direct goal updates, and the former
compensated-to-stationary aiming behavior on invalid shots. It does not change
the robust/legacy solver flags, PID, Motion Magic settings, or feed switch.

| Constant under `Constants.Turret` | Default | Effect |
| --- | --- | --- |
| `GOAL_DEADBAND_DEG` | 0.15 degrees | Hold the last issued goal until accumulated angle change exceeds this value. Zero disables this deadband. |
| `TARGET_SWITCH_CONFIRM_SEC` | 0.10 seconds | A new zone/target must remain selected for this time before switching. Zero removes the confirmation delay. |
| `PASSBACK_SWITCH_ADVANTAGE_METERS` | 0.15 meters | A new upper/lower passback target must be nearer in Y by more than this margin. |
| `PREVIOUS_SOLUTION_MAX_AGE_SEC` | 0.10 seconds | Maximum age since a fresh valid solver result for previous-shot revalidation. Reuse does not extend this deadline. |
| `STABILIZATION_MAX_SAMPLE_GAP_SEC` | 0.10 seconds | Reset target/goal history after a larger update gap or time reversal. |

Fresh valid mapped/direct shots remain preferred. If the new shot fails, a
recent previous shot can be reused only after recomputing the current predicted
release state, motion compensation, tilt compensation, mechanism limits, target
domain, and final trajectory geometry. The old field launch pitch is recovered
from its field vector; the old tilt-corrected hood angle is not applied twice.
This is previous-candidate validation, not the proposed hybrid search.

If neither candidate is valid, hold the current commanded turret goal and mark
the shot invalid. Do not jump back to the ordinary uncompensated aiming angle.
Holding a motor position does not preserve a valid trajectory: readiness-on
feeding still blocks. Readiness-off feeding remains the driver's chosen bypass.

The goal deadband works in continuous legal turret rotations and keeps the
selected wrap branch until a legal rewind is necessary. Real motion accumulates
relative to the last issued goal, so slow aiming changes are not discarded.
It adds no low-pass time delay to normal counterrotation. The final readiness
check also compares measured angle against the unfiltered solver goal, so
conditioning cannot make a misaligned raw aim count as aligned.

History resets on command restart/interruption, suppression, unavailable or stale
motion, unknown alliance, target/alliance changes, and relevant sample gaps.

Dashboard keys under `Turret/Stabilization/` show `Enabled`, `RawGoalDeg`,
`CommandedGoalDeg`, `CorrectionDeg`, `RevalidatedPrevious`, and `Status`.
Compare raw goal, issued goal, Motion Magic reference, and actual turret position
while driving. Existing general NetworkTables logging is disabled, so verify
recording before assuming these dashboard keys are in a saved log.

These defaults are starting values, not measured scoring or tracking results.
Keep the PID baseline and compare moving-goal RMS/peak error and scoring on the
robot. This change does not add the separate position/velocity tracking mode or
unmeasured kV/kA gains.

# Acceptance criteria — declare before the test

A test is INCOMPLETE when required criteria or conditions are missing. Criteria must be chosen before examining the result. Changing criteria creates a new revision; it does not retroactively turn a failure into a pass.

- Criteria ID / revision / date / approver:
- Hardware revision and applicable calibration/configuration:
- Mechanism or shooting mode:
- Practice or field qualification sought:
- Allowed voltage/temperature/load conditions:

## Provisional bench starting point

These are suggested engineering test criteria for a small turret step, **not measured facts or universal competition requirements**. Explicitly accept or replace every row before the test. Current code keeps the 0.5° bench diagnostic tolerance separate from its relaxed 2° shooting alignment tolerance; neither is a measured scoring requirement.

| Criterion | Suggested starting criterion | Accepted value / rationale |
|---|---|---|
| Bidirectional step coverage | ±15°, three valid repetitions each, without clipping | |
| Trial maximum duration | 3 s; timeout is FAIL/INVALID by cause | |
| Settling band | ±0.5° | |
| First sustained settling deadline | 1.0 s after step | |
| Continuous dwell before settled | 0.25 s | |
| Observation after settling | 1.0 s with no out-of-band departure | |
| Overshoot | ≤1.0° | |
| Steady-window RMS error | ≤0.25° | |
| Critical signal maximum age | ≤0.10 s, with no uncovered interval claimed as passing | |
| Maximum allowable gap between trial samples | 0.04 s for a 50 Hz qualification stream | |

These thresholds may be too strict or too loose for the real mechanism; use measured response and the launch error budget to revise them. Do not relax them merely to hide current limiting or mechanical trouble. A good encoder response is not proof of scoring.

## Mechanism-specific criteria

Do not apply turret degrees to hood motor rotations or flywheel RPS.

| Mechanism/test | Position/speed error | Settling/recovery deadline | Overshoot/droop | Hold interval | Approved operating envelope |
|---|---|---|---|---|---|
| Hood | | | | | |
| Flywheel unloaded | | | | | |
| Flywheel single Fuel | | | | | |
| Flywheel burst | | | | | |
| Intake extend/retract | | | | | |
| Moving turret track | | | | | |

## Shot and throughput criteria

| Mode/distance band | Target type | Minimum attempts per condition | Required makes or impact dispersion | Required throughput/recovery | Allowed motion |
|---|---|---:|---|---|---|
| Near stationary | | | | | |
| Mid stationary | | | | | |
| Far stationary | | | | | |
| Moving Hub | | | | | |
| Passback | | | | | |

Prefer at least 20 observed attempts per final representative condition and another session/battery when practical; report numerator/denominator and conditions. This is a screening suggestion, not proof of a precise success probability. A net impact cannot populate the Hub-makes column. Mark unobserved outcomes separately and include misses in denominators.

For a distance d and allowed lateral-error budget b, `atan(b/d)` gives a geometric angular-error budget. b must reserve room for localization, launch spread and target geometry; do not assign the entire opening to turret error or treat worst-case budgets as measured probability. Record the chosen allocation:

## Electrical/runtime/release criteria

- Minimum acceptable loaded voltage and maximum duration below warning margin, with rationale for the installed hardware:
- Required intake completion time and zero failures:
- Required shooter recovery/feed throughput:
- Temperature/CAN/source-age limits selected from device specs and verified conditions:
- Solver target: under 5 ms per evaluation on roboRIO; measured worst case and observation coverage required.
- Scheduler: no attributable 20 ms overruns; no sustained overruns under full load.
- Three consecutive 150 s trials: no DS brownouts, controller resets, failed intake movements or sustained loop overruns.
- Calibration/field/mode tests not available in current facility:

Decision: approved for this trial / needs revision. Signature or team role/date:

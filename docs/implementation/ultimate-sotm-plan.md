# Ultimate SOTM implementation contract

Status: approved plan; software implementation complete, hardware qualification pending. This document records accepted policy; implementation evidence is in `ultimate-sotm-implementation-report.md`.

## Objective and authority

Finish matches reliably, improve Hub and passback shot evaluation, diagnose turret oscillation and tracking lag, and collect evidence for current-limit selection. Preserve the existing working tree. The reference is the current edited calculator, not only the committed branch.

Read [the glossary](../../CONTEXT.md) and [shot evaluation ADR](../adr/0001-shared-shot-evaluation-contract.md), [vision/feed ADR](../adr/0002-vision-occlusion-and-feed-readiness.md), and [power ADR](../adr/0003-observe-power-before-changing-control.md). These are approved choices, not requests to reopen the interview.

## Accepted behavior

- Cover both Hub and passback; match calculator field target coordinates, including the actual calculator red Hub point and blue passback Y values 6.0/2.5. Do not preserve the Java-only 0.20 m red offset or blue Y values 7.4/0.6847.
- Port the full evaluation, not just the radial candidate sweep. Evaluate final compensated opening intersection, lip clearance, and Hub entry angle. Use target-plane crossing time instead of averaging incompatible horizontal and vertical times. Separate feasibility, mechanism readiness, and heuristic ranking.
- Keep existing launch calibration coefficients initially. Retain legacy selection only as explicit rollback; do not automatically switch to it after rejection by the new evaluator.
- Add a constants-controlled feed interlock, enabled for validation. Require final shooter speed, hood position, legal aligned turret, and valid shot. Preserve initial feed delays; on readiness loss stop feeding, then require 100 ms continuously ready before restarting the staged sequence.
- Right-stick click held together with the existing right-trigger shoot request is the proposed implementation default for manual readiness override. Override stays within finite bounded setpoints, enabled operation, and actuator protections; it does not invent a command when none exists. Autonomous uses the interlock without driver override.
- Vision age alone is warning-only, even beyond the current 0.25 s threshold. Use accepted capture timestamps for age and continue fused odometry. Nonfinite or disconnected critical control inputs remain invalid.
- Power work is logging-only initially: preserve current limits and existing drive caps. The hardware is REV PDH. PDH ID/channel mapping, roboRIO model, USB availability, and battery identity need setup verification, not invented values.
- Initial sampling budget: 50 Hz motor electrical data, 100 Hz turret motion, 10 Hz slow health, and 5 Hz dashboard summaries. Honor physical device update rates. Retain Phoenix logs and add synchronized WPILib DataLog records; prefer USB with internal fallback. Logging failure must not block control.
- Turret diagnostics distinguish final goal error, motion-profile tracking error, output saturation, and suspected current limiting. Provide stationary step metrics and moving tracking metrics; gains are measured on hardware, not guessed or auto-tuned in competition.
- Fix intake homing interruption/timeout behavior, check motor configuration results with bounded startup work, and remove duplicate drivetrain simulation updates. Do not silently redesign mechanical startup zeroing.

## Design tree and delivery order

1. Reliability priority -> observe confirmed brownouts -> REV PDH and motor logging -> offline analysis -> field-selected limits in a later change.
2. Current calculator authority -> shared evaluator contract -> Java Hub/passback port -> continuous feed readiness -> hardware qualification.
3. Vision occlusion under defense -> warning-only freshness -> degraded-pose records -> deliberate manual override for other readiness failures.
4. Oscillation and moving lag -> separate profile/goal errors -> Test-mode step tests -> physical gain/profile tuning.

Implementation packets: 00 baseline/contracts; 01 power recorder; 02 power analysis; 03 calculator evaluator; 04 Java port; 05 feed/pose policy; 06 turret diagnostics; 07 stability fixes; 08 integration and hardware qualification. Run sequentially by default; packet prerequisites identify safe alternative ordering. Do not start additional tasks or agents solely because packets exist.

## Validation and completion

At planning time the calculator passed 22 tests; Java compiled with 35 tests, of which three FieldTargetsTest cases failed due to the approved-to-change coordinates. Recheck rather than assume these counts remain current.

Require cross-language fixtures, independent trajectory checks, feed transition tests, sensor/logging failure tests, homing interruption tests, and a passing complete Java/calculator suite. Target solver duration below 5 ms on roboRIO with no attributable 20 ms loop overruns; desktop timings do not satisfy that requirement.

The team can perform full driving/shooting tests. Qualify through stationary calibration, moving shots, and three consecutive 150-second combined-load trials across representative serviceable batteries: no DS brownouts, resets, failed intake movements, or sustained loop overruns. Record scoring results, turret metrics, voltage/current, battery identity, and deployed configuration. Unit tests and simulated success do not authorize claims of competition qualification.

## Sources and limits

- [CTRE current limits](https://v6.docs.ctr-electronics.com/en/2025/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html): conceptual supply/stator distinction; verify exact APIs against installed Phoenix 26.1.2.
- [WPILib brownout diagnostics](https://docs.wpilib.org/en/latest/docs/software/roborio-info/roborio-brownouts.html).
- [WPILib power distribution](https://docs.wpilib.org/en/latest/docs/software/can-devices/power-distribution-module.html).

Calculator geometry and calibration are reference inputs, not independently certified field measurements. Aerodynamic modeling, a new automatic power controller, and numerical competition PID/current values are outside this first implementation. Any later departure should be explicit, measured, and reflected in these decisions.

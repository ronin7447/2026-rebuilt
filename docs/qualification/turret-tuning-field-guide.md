# Turret tuning field guide

For the complete bring-up and implementation sequence, use the [human field-readiness guide](../field-readiness/HUMAN-GUIDE.md). The following describes existing controls, not hardware-qualified operation; complete its reference/control and trial-lifecycle improvements before relying on qualification verdicts.

Use Test mode only. First confirm the forward zero, logical degrees, soft limits, mechanical freedom, and configuration health. Then assess profile following under the existing electrical limits before changing feedback gains.

The dashboard exposes `Turret/TestStepPositive15Deg` and `Turret/TestStepNegative15Deg` as commands to schedule and explicitly cancel. The current trial is position-bounded but has no timeout. Leaving Test mode also stops it and holds position; its current end action sets auto-aim suppression to false rather than restoring the previous state. [Packet 03](../field-readiness/ai-handoffs/03-control-qualification.md) fixes that lifecycle and adds qualification verdicts. No sweep runs at startup.

The diagnostics distinguish:

- final goal error (`GoalErrorDeg`);
- Motion Magic profile tracking error (`ProfileTrackingErrorDeg`);
- measured/profile velocity and motor output voltage;
- supply/stator current and CTRE-reported limiter faults;
- stationary settling time, overshoot, final bias, and sign changes;
- rolling moving-target RMS/peak goal and profile errors.

`CurrentLimitStatus=suspected` is deliberately not an authoritative limiter claim. Output saturation alone does not identify a bad kP value.

These metrics currently appear on the dashboard; general NetworkTables logging is disabled, so their presence in a saved log must be verified. [Packet 02](../field-readiness/ai-handoffs/02-session-recording-and-preflight.md) adds explicit diagnostic recording. The current first-settled metric alone does not establish sustained stability.

Tune in this order: validate zero/units and mechanics; assess profile tracking and electrical limits; tune stationary response; repeat while translating; repeat while rotating. Enter finite nonnegative gains and positive profile values, then press `Turret/ApplyTestTuning`. Application is rejected outside Test mode and every CTRE result is recorded. Do not copy trial values into production constants without a reviewed measured change.

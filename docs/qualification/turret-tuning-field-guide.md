# Turret tuning field guide

Use Test mode only. First confirm the forward zero, logical degrees, soft limits, mechanical freedom, and configuration health. Then assess profile following under the existing electrical limits before changing feedback gains.

The dashboard exposes `Turret/TestStepPositive15Deg` and `Turret/TestStepNegative15Deg` as commands to schedule and explicitly cancel. The current trial is position-bounded but has no timeout. Leaving Test mode also stops it and holds position; its current end action sets auto-aim suppression to false rather than restoring the previous state. No sweep runs at startup.

The diagnostics distinguish:

- final goal error (`GoalErrorDeg`);
- Motion Magic profile tracking error (`ProfileTrackingErrorDeg`);
- measured/profile velocity and motor output voltage;
- supply/stator current and CTRE-reported limiter faults;
- stationary settling time, overshoot, final bias, and sign changes;
- rolling moving-target RMS/peak goal and profile errors.

`CurrentLimitStatus=suspected` is deliberately not an authoritative limiter claim. Output saturation alone does not identify a bad kP value.

These metrics currently appear on the dashboard; general NetworkTables logging is disabled, so their presence in a saved log must be verified. The current first-settled metric alone does not establish sustained stability.

Tune in this order: validate zero/units and mechanics; assess profile tracking and electrical limits; tune stationary response; repeat while translating; repeat while rotating. Enter finite nonnegative gains and positive profile values, then press `Turret/ApplyTestTuning`. Application is rejected outside Test mode and every CTRE result is recorded. Do not copy trial values into production constants without a reviewed measured change.

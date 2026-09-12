# Turret tuning field guide

Use Test mode only. First confirm the forward zero, logical degrees, soft limits, mechanical freedom, and configuration health. Then assess profile following under the existing electrical limits before changing feedback gains.

Hold `Turret/TestStepPositive15Deg` or `Turret/TestStepNegative15Deg` from the dashboard to own the turret and run a bounded step. Releasing or leaving Test mode stops the trial, holds position, and restores auto-aim suppression state. No sweep runs at startup.

The diagnostics distinguish:

- final goal error (`GoalErrorDeg`);
- Motion Magic profile tracking error (`ProfileTrackingErrorDeg`);
- measured/profile velocity and motor output voltage;
- supply/stator current and CTRE-reported limiter faults;
- stationary settling time, overshoot, final bias, and sign changes;
- rolling moving-target RMS/peak goal and profile errors.

`CurrentLimitStatus=suspected` is deliberately not an authoritative limiter claim. Output saturation alone does not identify a bad kP value.

Tune in this order: validate zero/units and mechanics; assess profile tracking and electrical limits; tune stationary response; repeat while translating; repeat while rotating. Enter finite nonnegative gains and positive profile values, then press `Turret/ApplyTestTuning`. Application is rejected outside Test mode and every CTRE result is recorded. Do not copy trial values into production constants without a reviewed measured change.

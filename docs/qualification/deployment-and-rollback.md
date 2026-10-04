# Deployment, operation, and rollback

Start with the [complete human field-readiness guide](../field-readiness/HUMAN-GUIDE.md). The September 2026 implementation is still hardware-unverified, and a changed flywheel needs new calibration. This short reference does not replace its prerequisites.

## Before enabling

1. Record revision, configuration ID, battery label, PDH ID/channel map, roboRIO version, CAN health, firmware, and log storage.
2. Confirm every essential `Health/Config/*/Applied` value and `PowerLogger/Healthy`.
3. Confirm turret/hood physical references. Intake homing exists as a command but has no dedicated operator binding in the current code; implement the supervised action and feedback-validity fixes in [packet 01](../field-readiness/ai-handoffs/01-control-and-reference-integrity.md) before following its homing procedure. A constructor encoder reset is not a confirmed physical reference.
4. Verify finite Hub/passback setpoints and `Feed/Blockers` while unloaded.

## Driver behavior

**Calibration-branch exception:** `launch-speed-calibration` currently leaves normal right-trigger shooting unbound and uses A/B for fixed-angle calibration. The normal controls below do not describe that branch's active bindings. See the [pass-back retest guide](passback-check.md) before testing automatic passing.

Hold right trigger to request shooting. Automatic feed requires the final roller speed, hood position, legal aligned turret, a current feasible shot result, and valid bounded setpoints. Vision age appears as `Feed/VisionDegradedWarning` but does not block a fused-odometry shot.

While holding right trigger, hold right-stick click for momentary manual readiness override. Releasing either control ends it. Override cannot bypass disabled operation, nonfinite/out-of-range setpoints, or turret actuator protections. It never creates a shoot command by itself. Autonomous has no override.

## Rollback

The robust evaluator is selected by `Constants.ShotMap.USE_ROBUST_SOTM`. `USE_LEGACY_SOLVER_ROLLBACK` is an explicit whole-mode rollback switch; rejected robust shots never silently invoke legacy logic. `FEED_INTERLOCK_ENABLED=false` bypasses full readiness gating but still checks enabled operation, live shooter-speed readiness, bounded setpoints and turret legality; it is not an exact restoration of the historical one-time speed latch. Record any rollback in the run sheet and configuration ID. A legacy model with an old flywheel calibration is not a verified fallback for new hardware.

Do not deploy as competition-qualified until the measured gates in `ultimate-sotm-run-sheet.md` pass.

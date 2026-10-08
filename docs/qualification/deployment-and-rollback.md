# Deployment, operation, and rollback

Start with the [complete human field-readiness guide](../field-readiness/HUMAN-GUIDE.md). The September 2026 implementation is still hardware-unverified, and a changed flywheel needs new calibration. This short reference does not replace its prerequisites.

## Before enabling

1. Record revision, configuration ID, battery label, PDH ID/channel map, roboRIO version, CAN health, firmware, and log storage.
2. Confirm every essential `Health/Config/*/Applied` value and `PowerLogger/Healthy`.
3. Confirm turret/hood physical references. Intake homing exists as a command but has no dedicated operator binding in the current code; implement the supervised action and feedback-validity fixes in [packet 01](../field-readiness/ai-handoffs/01-control-and-reference-integrity.md) before following its homing procedure. A constructor encoder reset is not a confirmed physical reference.
4. Verify finite Hub/passback setpoints and `Feed/Blockers` while unloaded.

## Driver behavior

Hold right trigger to request shooting. Automatic feed requires the final roller speed, hood position, legal aligned turret, a current feasible shot result, valid bounded setpoints, and fresh measured motion inside the [provisional shooting envelope](../implementation/ultimate-sotm-motion-envelope.md). This motion gate also applies to autonomous; faster autonomous paths may pause feeding. Vision age appears as `Feed/VisionDegradedWarning` but does not block a fused-odometry shot.

Teleop shooting retains the existing reduced speed caps and adds translation-vector and angular command ramps. Entering shooting from a faster command decelerates toward those caps; physical motion must satisfy feed readiness. These ramps act on the default joystick drive command. Autonomous and autopilot motion use their own controllers and are checked by the measured-motion feed gate.

While holding right trigger, hold right-stick click for momentary manual readiness override, including motion-envelope readiness. Releasing either control ends it. Override cannot bypass disabled operation, nonfinite/out-of-range setpoints, or turret actuator protections. It never creates a shoot command by itself. Autonomous has no override. Qualification shots must have override inactive.

## Rollback

The robust evaluator is selected by `Constants.ShotMap.USE_ROBUST_SOTM`. `USE_LEGACY_SOLVER_ROLLBACK` is an explicit whole-mode rollback switch; rejected robust shots never silently invoke legacy logic. `FEED_INTERLOCK_ENABLED=false` bypasses full readiness gating but still checks enabled operation, live shooter-speed readiness, bounded setpoints and turret legality; it is not an exact restoration of the historical one-time speed latch. Record any rollback in the run sheet and configuration ID. A legacy model with an old flywheel calibration is not a verified fallback for new hardware.

Do not deploy as competition-qualified until the measured gates in `ultimate-sotm-run-sheet.md` pass.

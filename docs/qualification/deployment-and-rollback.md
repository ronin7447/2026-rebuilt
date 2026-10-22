# Deployment, operation, and rollback

Start with the [complete human field-readiness guide](../field-readiness/HUMAN-GUIDE.md). The September 2026 implementation is still hardware-unverified, and a changed flywheel needs new calibration. This short reference does not replace its prerequisites.

## Before enabling

1. Record revision, configuration ID, battery label, PDH ID/channel map, roboRIO version, CAN health, firmware, and log storage.
2. Confirm every essential `Health/Config/*/Applied` value and `PowerLogger/Healthy`.
3. Confirm turret/hood physical references. Intake homing exists as a command but has no dedicated operator binding in the current code; implement the supervised action and feedback-validity fixes in [packet 01](../field-readiness/ai-handoffs/01-control-and-reference-integrity.md) before following its homing procedure. A constructor encoder reset is not a confirmed physical reference.
4. Verify finite Hub/passback setpoints and `Feed/Blockers` while unloaded.

## Driver behavior

Hold right trigger to request shooting. Automatic feed requires roller speed inside the relaxed readiness band, hood position within 0.08 motor rotations (about 0.93°), turret alignment within 2°, a current feasible shot result, valid bounded setpoints, and fresh measured motion inside the [provisional shooting envelope](../implementation/ultimate-sotm-motion-envelope.md). This motion gate also applies to autonomous. Vision age appears as `Feed/VisionDegradedWarning` but does not block a fused-odometry shot.

Roller readiness starts within the larger of 3 RPS or 10% of requested magnitude and remains ready within the larger of 5 RPS or 15%. A 25 RPS request with 26.5 RPS feedback starts without a stability dwell. Invalid feedback, disable, wrong direction, or a large target change resets readiness. Normal feed starts the outtake at 50 ms and indexer at 75 ms after readiness, quantized by the scheduler loop. After readiness loss, recovery requires 50 ms continuously ready, then the 75 ms staging sequence.

Teleop shooting uses `Constants.Drivebase.SHOOT_LINEAR_SPEED_LIMIT_SCALE=0.70` (70% of normal full-stick translation) and `SHOOT_ANGULAR_SPEED_LIMIT_SCALE=0.50` (50% of normal full-stick rotation). Set either to `1.0` for no shooting speed reduction. The measured feed speed limits derive from those same settings with 20% headroom. Command ramps are 2 m/s² and 180°/s²; measured acceleration limits allow 50% headroom. Entering shooting from a faster command decelerates toward the caps. These ramps act on the default joystick drive command. Autonomous and autopilot motion use their own controllers and are checked by the measured-motion feed gate. Solver domain/actuator limits still apply to automatic shots even if the drive percentages are raised.

While holding right trigger, hold right-stick click to **force feed immediately**. This skips auto-aim validity, roller speed, hood/turret alignment, motion readiness, and all staging/recovery delays. It works with the current bounded actuator commands even when there is no mapped auto-aim solution, including manual calibration. Auto aim continues when active; override does not invent or overwrite launch commands. Disabled operation, a disabled/unconfigured shooter roller, and invalid actuator setpoints still prevent force feed. Motor-controller protections remain active. Releasing right-stick click restores automatic readiness and recovery staging; releasing right trigger stops the shoot command. Autonomous has no override. Qualification shots must have override inactive.

## Rollback

The robust evaluator is selected by `Constants.ShotMap.USE_ROBUST_SOTM`. `USE_LEGACY_SOLVER_ROLLBACK` is an explicit whole-mode rollback switch; rejected robust shots never silently invoke legacy logic. `FEED_INTERLOCK_ENABLED=false` bypasses full readiness gating but still checks enabled operation, live shooter-speed readiness, bounded setpoints and turret legality; it is not an exact restoration of the historical one-time speed latch. Record any rollback in the run sheet and configuration ID. A legacy model with an old flywheel calibration is not a verified fallback for new hardware.

Do not deploy as competition-qualified until the measured gates in `ultimate-sotm-run-sheet.md` pass.

# Deployment, operation, and rollback

## Before enabling

1. Record revision, configuration ID, battery label, PDH ID/channel map, roboRIO version, CAN health, firmware, and log storage.
2. Confirm every essential `Health/Config/*/Applied` value and `PowerLogger/Healthy`.
3. Confirm turret/hood forward references and run intake homing. Only `SUCCESS` changes the intake encoder reference; timeout/interruption retains it.
4. Verify finite Hub/passback setpoints and `Feed/Blockers` while unloaded.

## Driver behavior

Hold right trigger to request shooting. Automatic feed requires the final roller speed, hood position, legal aligned turret, a current feasible shot result, and valid bounded setpoints. Vision age appears as `Feed/VisionDegradedWarning` but does not block a fused-odometry shot.

While holding right trigger, hold right-stick click for momentary manual readiness override. Releasing either control ends it. Override cannot bypass disabled operation, nonfinite/out-of-range setpoints, or turret actuator protections. It never creates a shoot command by itself. Autonomous has no override.

## Rollback

The robust evaluator is selected by `Constants.ShotMap.USE_ROBUST_SOTM`. `USE_LEGACY_SOLVER_ROLLBACK` is an explicit whole-mode rollback switch; rejected robust shots never silently invoke legacy logic. `FEED_INTERLOCK_ENABLED=false` restores the former shooter-speed staging policy while retaining numeric and actuator protections. Record any rollback in the run sheet and configuration ID.

Do not deploy as competition-qualified until the measured gates in `ultimate-sotm-run-sheet.md` pass.

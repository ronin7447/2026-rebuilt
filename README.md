# 2026-rebuilt
Code repository for the FRC 2026 season, REBUILT, for Team 7447 Rōnin Robotics.

## References

- [Turret smoothing and feed readiness](docs/turret-target-stabilization.md)
- [Turret tuning field guide](docs/qualification/turret-tuning-field-guide.md)
- [Power and health log signals](docs/implementation/power-signal-dictionary.md)
- [Camera mount visibility analysis](docs/vision/camera-mount-analysis/README.md)
- [Shot-map trial profiles](tools/shot-calculator/FAR_SHOT_TRIALS.md)

Current settings live in `src/main/java/frc/robot/constants/Constants.java`.
A is a hold-to-zero hood override; the former A/B calibration bindings are not active.

## PathPlanner bump-exit autopilot

Add a sequential **Named Command** step named `AutopilotToBumpExit` to a PathPlanner Auto. It drives to absolute blue-origin field pose `(14.0 m, 5.5 m, 45 degrees)` on either alliance, with maximum velocity `4.5 m/s`, maximum acceleration `5.0 m/s²`, and zero goal-end velocity. It stops on completion or interruption. Do not use it as an event marker inside a running path: it owns the drivetrain.

The callable Java function is `autopilot.goToBumpExit()`. The target is in `FieldDestinationConstants.BUMP_EXIT_TARGET`; linear limits are `Constants.Drivebase.BUMP_AUTOPILOT_MAX_SPEED_MPS` and `BUMP_AUTOPILOT_MAX_ACCEL_MPS2`. Existing angular limits are 540 degrees/s and 720 degrees/s². Pathfinding follows the deployed navgrid, so the route depends on the starting pose and obstacles; a particular bump crossing is not guaranteed. These are trajectory limits, not a measured bump-crossing capability.

# Pass-back investigation and robot retest

Date: 2026-10-03. Investigated checkout: `launch-speed-calibration`, based on `c49a6ec`. The changes described here are software-tested, **not robot-tested or competition-qualified**. The new flywheel still needs calibration. Start with the [human field-readiness guide](../field-readiness/HUMAN-GUIDE.md) for mechanism-reference and calibration prerequisites.

## What was found and changed

The normal pass-back solver is not fixed at 70°. The A-button `LaunchSpeedCalibrationCommand` deliberately commands `Constants.LaunchSpeedCalibration.LAUNCH_ANGLE_DEG = 70.0`, holds turret yaw at zero, and suppresses auto-aim. In this checkout normal right-trigger shooting is unbound. A calibration shot therefore cannot demonstrate automatic pass-back behavior. These controls have been preserved.

There was a separate, reproducible moving-pass-back error in both Java and the TypeScript reference evaluator. Candidate selection considered range and motion toward/away from the target, but ignored sideways motion. The later aiming calculation spent some horizontal launch speed cancelling sideways motion; this reduced downrange speed and changed flight time. Its final trajectory check then rejected shots that a different hood/speed combination could make.

Pass-back candidate selection now uses the compensated downrange speed:

`sqrt(horizontalLaunchSpeed² - sidewaysRobotSpeed²) + towardTargetRobotSpeed`

It selects hood and speed against that trajectory using continuous range/velocity inputs. It retains the existing bounded search, mechanism limits, final trajectory validation, and feed interlocks. Hub polynomial selection and close-range Hub selection remain unchanged. Unreachable pass-back requests report `no-feasible-passback-shot`; inputs outside the configured pass-back domain report `outside-passback-domain`.

This fixes a software cause of rejected moving passes. It does not establish that the team's observed physical failure had that cause.

## Diagnose the reported 70° problem first

Record which button was pressed and whether `LaunchSpeedCalibration/Running` was true. Describe whether Fuel never fed, fell short, overshot, or missed sideways. Save a video and the robot log, including the revision and physical flywheel configuration.

The current calibration equation is:

`hood motor rotations = 0.0857913 × launch angle in degrees - 7.49442`

Thus a 70° launch-angle request maps to approximately **-1.489029 motor rotations**. The current allowed hood range corresponds to approximately **60.93–80° launch angle**. Launch angle means the Fuel's initial direction above horizontal; a measured hood panel angle is not automatically the same quantity.

1. Unload Fuel and verify the known physical hood reference using the team's reference procedure. The constructor and Start-button encoder reset do not measure the physical reference. Do not reset at an arbitrary position or widen travel limits to make a shot work.
2. Compare the requested hood position with actual motor position and the physical mechanism. `HoodPosition` is overwritten with the **target** in periodic telemetry; it is not proof the hood reached that position. Read actual position from the hood Talon signal (`ShooterSubsystem.getHoodPosition()`), or the corresponding measured motor-position log signal.
3. For normal auto-aim, record `Turret/RecommendedLaunchPitchDeg` and `Turret/MappedHoodPosition`. For calibration mode record `LaunchSpeedCalibration/HoodTargetRot` instead; suppressed auto-aim telemetry is not the active calibration command.
4. If the command is 70° because calibration is running, this is expected. If normal pass-back computes approximately 61° but the actual exit direction stays around 70°, investigate reference, hood tracking, and calibration before tuning projectile speed. If the motor reaches its request but video shows the wrong exit angle, refit the angle calibration.

Under a simplified no-drag model, changing launch angle from about 61° to 70° at the same speed can substantially shorten a pass. This is a plausible explanation for short passes, not evidence that it happened on this robot.

## Before attempting an automatic pass

1. Incorporate this solver fix into the intended normal-controls deployment. Merely switching to the existing `ultimate-sotm` branch does not carry these working-tree edits with it. Have a coding agent integrate the change and verify the normal right-trigger binding to `createDriverShootCommand()`; retain the explicit calibration mode separately if needed. Do not use the A-button calibration command as an automatic pass-back test. Review the control layout with the driver before enabling.
2. Complete reference-integrity checks and the new-flywheel measurements in the human guide. Record measured Fuel exit direction, flight time, distance, hood motor position, and roller RPS. Old coefficients are not validated for a changed flywheel.
3. Unloaded, confirm correct alliance, trustworthy field pose, and selected target coordinates. The current targeting policy chooses `NEUTRAL_BACK` in the neutral zone, `OPPONENT_PASS_BACK` in the opponent zone, and `HUB` in the robot's own alliance zone. Opponent-zone passes target neutral-zone points, not the far alliance wall. Never falsify field pose during a live shot just to select pass-back.
4. Check `Turret/TargetMode`, `Turret/TargetX`, `Turret/TargetY`, and `Turret/ShotModel`. A valid normal pass should use `ReferencePassback`. A rejected shot may have no valid model/setpoints; inspect `Turret/RobustShotReason`.
5. Verify unloaded that feeding requires a valid solution, actual hood/roller readiness, and legal aligned turret. Do not use readiness override to validate the solver.

## Controlled shooting sequence

Use a net and a clear, measured shot corridor with enough ceiling clearance for the entire arc. The model checks the destination, not intervening robots, field structures, or room obstacles. Define an acceptable landing area before the trial; the model's 0.24 m target-height tolerance is not a measured landing-accuracy guarantee.

1. **Stationary:** select the shortest practical pass-back shot supported by the real zone/target geometry and test one Fuel at a time. A neutral-zone pass will usually be longer than a 2 m bench shot. Record at least five shots at one location before extending distance. Compare video/landing results with predicted flight time; stop and resolve a systematic miss before adding motion.
2. **Change distance:** repeat at additional intended passing locations within the available clear corridor. Record measured range from the shooter release point, target height (configured pass-back height is 0.5 m), mean landing error, spread, and misses. A pass-back target plane at 0.5 m is not the ground landing point.
3. **Slow sideways travel:** after stationary results are acceptable, begin around 0.5 m/s with steady velocity, in both directions. Keep acceleration and turning low. Increase speed only after measuring acceptable results; the desktop 2 m/s regression is not an approved physical operating limit.
4. **Toward/away travel and turning:** qualify these separately after lateral tests. Acceleration, release delay, turret motion and pose errors require their own measured coverage. If misses appear only during changes in speed, retain steady/slow passing while collecting release-timing and motion data.
5. **Combined-load trials:** repeat the useful pass cases with the expected drivetrain/intake load while logging battery voltage, current, roller recovery, CAN health and roboRIO loop/solver duration. Complete the existing field-readiness endurance gates before competition qualification.

For every trial save revision, coefficients, battery ID, alliance, real pose/target, commanded and measured hood position, roller speed, video, landing observations, and these dashboard/log values:

- `Turret/RobustDistanceMeters`, `Turret/RobustRadialVelocityMps`, `Turret/RobustTangentialVelocityMps`
- `Turret/RecommendedLaunchPitchDeg`, `Turret/MappedHoodPosition`, `Turret/MappedRollerRps`, `Turret/FlightTimeSec`
- `Turret/RobustShotReason`, `Turret/RobustSolverDurationMs`
- `Feed/ShootRequested`, `Feed/Permitted`, `Feed/Reason`, `Feed/Blockers`

## What to do when a test fails

| Observation | Next action |
|---|---|
| A commands 70° and turret stays forward | Calibration mode is active. Use the reviewed normal-controls deployment for automatic pass-back. |
| Normal right trigger does nothing on this checkout | It is unbound. Integrate/review normal controls first; this is not solver evidence. |
| `no-feasible-passback-shot` | Reduce range/motion and inspect actual inputs. A stationary 12 m request currently has no solution within the modeled hood/speed limits. Do not force feed or increase limits without physical evidence. |
| `outside-passback-domain` | Verify range/velocity/pose. The configured domain is 0.4–12 m and radial velocity ±2.5 m/s; being inside it does not guarantee a feasible shot. |
| `tilt-corrected-hood-out-of-range` | Move to level ground/reduce tilt. Candidate selection does not search alternate hood settings after this rejection. |
| Valid trajectory but feed blocked | Read `Feed/Blockers`; inspect actual speed recovery, hood tracking, turret alignment/limits and command state. |
| Feeds but consistently short/long when stationary | Check actual exit angle, flywheel calibration, target height, pose/range and wheel slip. Keep motion out of the test until resolved. |
| Only moving shots miss | Check measured chassis velocity, release latency and turret tracking; collect steady-motion cases before acceleration cases. |
| Good desktop behavior but loop overruns on robot | Capture solver and robot-loop durations under combined load before approving moving passes. |

## Software evidence

- Java: `JAVA_HOME=/Users/junyu/wpilib/2026/jdk ./gradlew test --offline` on the development machine: **67 tests passed**. Other computers should use their installed WPILib JDK location.
- Calculator: `npm test && npm run build` in `tools/shot-calculator`: **29 tests passed**, TypeScript/Vite build passed.
- New regression cases include stationary angle selection, 2/4/6/8 m passes with ±2 m/s lateral motion, simultaneous ±0.75 m/s radial and 1.5 m/s lateral motion, and unreachable/out-of-domain rejection. Tests independently reconstruct the commanded ballistic flight. Three new Java tests failed before the fix and passed afterward.
- Shared Java/TypeScript fixtures include both lateral directions with the robot's release prediction and pivot offset. Existing Hub fixture expectations remain unchanged.

Relevant implementation: [Java solver](../../src/main/java/frc/robot/util/sotm/RobustShotModel.java), [Java regressions](../../src/test/java/frc/robot/util/sotm/RobustShotModelTest.java), [calculator evaluator](../../tools/shot-calculator/src/math/evaluator.ts), [candidate search](../../tools/shot-calculator/src/math/trajectory.ts), and [shared fixtures](../../shared/shot-evaluator-fixtures-v1.csv).

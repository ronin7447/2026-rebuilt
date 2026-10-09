# Ultimate SOTM: release prediction and scoring envelope

Date: 2026-10-08. Branch: `ultimate-sotm`. Configuration: `ultimate-sotm-v3-throughput`.
Software verified; hardware qualification pending.

The objective is useful scoring throughput. The user explicitly accepts some misses in exchange for faster feeding and more freedom of motion. The current settings loosen alignment and flywheel readiness and provide immediate manual force feed. Automatic shots still require a feasible solver result; driver override can feed using existing actuator commands without that result. The September plan's instruction to preserve drive caps described the earlier implementation.

## What already accounted for acceleration

The robust solver already used pose, field velocity, filtered field acceleration, angular velocity, angular acceleration, and roll/pitch. For latency `τ`, release position is `p + vτ + ½aτ²`, release velocity is `v + aτ`, release heading is `θ + ωτ + ½ατ²`, and release angular velocity is `ω + ατ`. The offset turret pivot contributes `ω × offset` to inherited launch velocity. The solver then compensates radial/tangential motion, transforms the launch vector through the predicted robot orientation, and commands turret yaw, hood position, and roller RPS. Final evaluation checks opening intersection, front lip clearance, and descending entry angle.

Robot acceleration affects the state at departure. After departure, the projectile model applies gravity; subsequent robot acceleration cannot push already-released Fuel. The current trajectory model does not include aerodynamic drag, spin, variable exit location, changing roll/pitch during latency, or jerk. Existing launch calibration and the provisional 0.14 s latency still need hardware measurement.

## Changes

One continuously updated drivetrain motion estimate now serves both aiming and feed readiness. Derivatives use successive Phoenix odometry timestamps and field-frame velocities. Repeated timestamps preserve the estimate; missing, nonfinite, invalid-odometry, reversed-time, or excessively separated samples cannot authorize automatic feed. A fresh pair is required after a history gap. Aiming includes sample age in its prediction horizon, so release is predicted from the observation time through evaluation and launch latency.

Feed readiness checks measured current and predicted release speed, raw and filtered linear acceleration, current and predicted angular speed, and raw and filtered angular acceleration. Raw magnitudes prevent filtering or clamping from hiding sudden motion. Readiness loss stops commanded feed outputs immediately; automatic recovery requires 50 ms continuously ready followed by 50 ms outtake staging and another 25 ms to the indexer. Force feed bypasses all of these delays. Fuel already in transport can still depart after commanded feed stops, which is why latency and transient trials matter.

| Quantity | Provisional limit | Where enforced |
|---|---:|---|
| Translation speed | 2.0748 m/s | Automatic feed, current and predicted release vector magnitude; 120% of the configured shooting command cap |
| Translation acceleration | 3.0 m/s² | Automatic feed, raw and filtered vector magnitude; 150% of the configured command ramp |
| Angular speed | 194.4°/s | Automatic feed, current and predicted release; 120% of the configured shooting command cap |
| Angular acceleration | 270°/s² | Automatic feed, raw and filtered magnitude; 150% of the configured command ramp |
| Motion sample age | 0.05 s | Aiming and automatic feed |
| Teleop shooting translation command | 1.729 m/s | 70% of normal full-stick translation; vector cap includes diagonal input |
| Teleop shooting angular command | 162°/s | 50% of normal full-stick rotation |
| Teleop shooting translation command ramp | 2.0 m/s² | Default joystick drive, including direction changes and deceleration |
| Teleop shooting angular command ramp | 180°/s² | Default joystick drive |

These numbers are engineering starting settings, not a measured accuracy envelope. Joystick ramps regulate requested motion; measured-motion readiness checks the physical response. Entry from a higher speed ramps down rather than stepping instantly to the lower cap. Autonomous and autopilot controllers retain their own motion commands; the feed gate pauses shooting outside this envelope. The two `Constants.Drivebase.SHOOT_*_SPEED_LIMIT_SCALE` values are fractions of normal full-stick speeds: `0.70` means 70%, and `1.0` removes the shooting speed reduction. Feed speed limits derive from these same settings, with configurable headroom, so increasing the percentages does not leave an unrelated fixed feed cap. Solver domain and actuator limits still apply to automatic shots. Manual force feed and interlock-off operation must be recorded separately from automatic scoring.

## Throughput readiness and force feed

Turret shooting alignment is ±2°. Bench diagnostic settling remains ±0.5°. Hood readiness is ±0.08 motor rotations, about ±0.93° of calibrated launch elevation.

Flywheel readiness starts within the larger of ±3 RPS or ±10% of target magnitude, and stays ready within the larger of ±5 RPS or ±15%. Thus 25 RPS requested with 26.5 RPS measured starts immediately without a stability dwell. The wider running band prevents normal variation from repeatedly resetting feed staging. Bad/stale feedback, disable, a zero target, wrong rotation direction, and large target changes reset the readiness latch.

Hold right trigger and right-stick click to force feed: both feed motors start on the next command execution, bypassing solution validity, roller speed, hood/turret alignment, motion limits, and all staging/recovery delays. This uses current finite, bounded actuator commands even when no solver mapping exists. Auto aim continues when active; override does not overwrite manually requested launch settings or select a new fallback shot. Robot disabled state, a disabled/unconfigured shooter roller, and invalid actuator setpoints still prevent feed. Motor-controller protections remain active. Release the click to resume automatic readiness/recovery; release right trigger to stop shooting. Autonomous has no override.

## Verification and hardware acceptance

The Java tests cover release position/velocity/heading, field-frame derivatives during rotation, duplicate/gapped/stale/nonfinite observations, hidden acceleration spikes, predicted speed limits, diagonal command magnitude, direction reversal, and feed stop/recovery. An independent trajectory reconstruction starts from the commanded turret, hood, and RPS and verifies an accelerating, rotating, tilted shot crosses the target line inside the height tolerance, clears the lip, and meets the descending entry-angle requirement. Existing shared calculator fixtures remain unchanged.

Use [the acceptance template](../field-readiness/templates/02-acceptance-criteria.md) to declare the required makes/attempts before testing. Record actual Hub entries, misses, and unobserved outcomes separately for near/mid/far range, toward/away motion, lateral motion in both directions, acceleration/deceleration, and combined rotation. Include transitions into shooting and reversals, both alliances, representative batteries, and repeat-session evidence. A net impact or simulated trajectory cannot count as a Hub make. Keep manual override inactive.

Measure actual Fuel departure latency and spread; calibrate hood angle and RPS-to-exit-speed for the installed flywheel. Retrieve logs with `Power/FeedBlockers`, shot-motion age/raw/filtered derivatives, actuator tracking, and battery data. Inspect scoring and the proportion of requested shooting time blocked by the motion gate: limits that prevent most shots are not a useful final approach. Raise throughput or motion limits only where measured scoring remains acceptable. Complete the existing roboRIO timing and combined-load qualification gates as well.

## Software evidence

- `./gradlew test --offline`: 75 tests passed, including signed flywheel hysteresis, the 25/26.5 RPS startup and fluctuation regression, immediate force feed without a solver result, override release, and disabled/invalid-command protections, alongside existing trajectory/motion tests.
- The preceding motion implementation passed 28 shot-calculator tests and its production build; this follow-up changes Java readiness/control policy and leaves calculator code and shared trajectory fixtures unchanged.
- `git diff --check`: passed.

No robot deployment or hardware scoring test was performed by this change. Competition accuracy remains unqualified.

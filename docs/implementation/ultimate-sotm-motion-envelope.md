# Ultimate SOTM: release prediction and scoring envelope

Date: 2026-10-08. Branch: `ultimate-sotm`. Configuration: `ultimate-sotm-v2-motion`.
Software verified; hardware qualification pending.

The minimum useful result is Fuel entering the Hub. A feasible trajectory and aligned mechanisms are prerequisites; neither establishes measured scoring accuracy. Limiting speed and acceleration during shooting is an acceptable operating tradeoff for repeatable scoring. This follow-up implements the user's requested motion limits; the September plan's instruction to preserve drive caps described the earlier implementation.

## What already accounted for acceleration

The robust solver already used pose, field velocity, filtered field acceleration, angular velocity, angular acceleration, and roll/pitch. For latency `τ`, release position is `p + vτ + ½aτ²`, release velocity is `v + aτ`, release heading is `θ + ωτ + ½ατ²`, and release angular velocity is `ω + ατ`. The offset turret pivot contributes `ω × offset` to inherited launch velocity. The solver then compensates radial/tangential motion, transforms the launch vector through the predicted robot orientation, and commands turret yaw, hood position, and roller RPS. Final evaluation checks opening intersection, front lip clearance, and descending entry angle.

Robot acceleration affects the state at departure. After departure, the projectile model applies gravity; subsequent robot acceleration cannot push already-released Fuel. The current trajectory model does not include aerodynamic drag, spin, variable exit location, changing roll/pitch during latency, or jerk. Existing launch calibration and the provisional 0.14 s latency still need hardware measurement.

## Changes

One continuously updated drivetrain motion estimate now serves both aiming and feed readiness. Derivatives use successive Phoenix odometry timestamps and field-frame velocities. Repeated timestamps preserve the estimate; missing, nonfinite, invalid-odometry, reversed-time, or excessively separated samples cannot authorize automatic feed. A fresh pair is required after a history gap. Aiming includes sample age in its prediction horizon, so release is predicted from the observation time through evaluation and launch latency.

Feed readiness checks measured current and predicted release speed, raw and filtered linear acceleration, current and predicted angular speed, and raw and filtered angular acceleration. Raw magnitudes prevent filtering or clamping from hiding sudden motion. Readiness loss stops commanded feed outputs immediately; recovery still requires 100 ms continuously ready followed by the existing staged feed delays. Fuel already in transport can still depart after commanded feed stops, which is why latency and transient trials matter.

| Quantity | Provisional limit | Where enforced |
|---|---:|---|
| Translation speed | 2.0 m/s | Automatic feed, teleop and autonomous, current and predicted release vector magnitude |
| Translation acceleration | 2.0 m/s² | Automatic feed, raw and filtered vector magnitude |
| Angular speed | 90°/s | Automatic feed, current and predicted release |
| Angular acceleration | 180°/s² | Automatic feed, raw and filtered magnitude |
| Motion sample age | 0.05 s | Aiming and automatic feed |
| Teleop shooting translation command | 0.494 m/s | Existing 20% shooting cap; now a vector cap, including diagonal input |
| Teleop shooting angular command | 48.6°/s | Existing 15% shooting cap |
| Teleop shooting translation command ramp | 1.0 m/s² | Default joystick drive, including direction changes and deceleration |
| Teleop shooting angular command ramp | 90°/s² | Default joystick drive |

These numbers are engineering starting settings, not a measured accuracy envelope. Joystick ramps regulate requested motion; measured-motion readiness checks the physical response. Entry from a higher speed ramps down rather than stepping instantly to the lower cap. Autonomous and autopilot controllers retain their own motion commands; the feed gate pauses shooting outside this envelope. Adjust paths or limits deliberately after testing. Existing momentary manual override can bypass motion readiness, and the explicit interlock-off rollback also bypasses it; neither setting qualifies automatic scoring.

## Verification and hardware acceptance

The Java tests cover release position/velocity/heading, field-frame derivatives during rotation, duplicate/gapped/stale/nonfinite observations, hidden acceleration spikes, predicted speed limits, diagonal command magnitude, direction reversal, and feed stop/recovery. An independent trajectory reconstruction starts from the commanded turret, hood, and RPS and verifies an accelerating, rotating, tilted shot crosses the target line inside the height tolerance, clears the lip, and meets the descending entry-angle requirement. Existing shared calculator fixtures remain unchanged.

Use [the acceptance template](../field-readiness/templates/02-acceptance-criteria.md) to declare the required makes/attempts before testing. Record actual Hub entries, misses, and unobserved outcomes separately for near/mid/far range, toward/away motion, lateral motion in both directions, acceleration/deceleration, and combined rotation. Include transitions into shooting and reversals, both alliances, representative batteries, and repeat-session evidence. A net impact or simulated trajectory cannot count as a Hub make. Keep manual override inactive.

Measure actual Fuel departure latency and spread; calibrate hood angle and RPS-to-exit-speed for the installed flywheel. Retrieve logs with `Power/FeedBlockers`, shot-motion age/raw/filtered derivatives, actuator tracking, and battery data. Inspect scoring and the proportion of requested shooting time blocked by the motion gate: limits that prevent most shots are not a useful final approach. Raise throughput or motion limits only where measured scoring remains acceptable. Complete the existing roboRIO timing and combined-load qualification gates as well.

## Software evidence

- `./gradlew test --offline`: 67 Java tests passed, including new release/motion/feed regression cases.
- Shot calculator: 28 tests passed; production build passed.
- `git diff --check`: passed.

No robot deployment or hardware scoring test was performed by this change. Competition accuracy remains unqualified.

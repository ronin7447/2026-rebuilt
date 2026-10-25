# Turret smoothing and feed readiness

Settings live in `Constants.java`; rebuild and deploy after editing. Dashboard values report the deployed configuration.

## Aim conditioning

- `Turret.AIM_SMOOTHING_ENABLED=true`: exponential smoothing of field-relative aim corrections.
- `Turret.AIM_SMOOTHING_TIME_CONSTANT_SEC=0.060`: larger values smooth more but add tracking delay. Zero bypasses filtering.
- Chassis counterrotation and the solver's predicted-release/local-frame conversion remain outside the filtered correction.
- Filtering resets on target/alliance changes, invalid shots, command reset/suppression, and long or reversed update intervals.
- `Turret.TARGET_STABILIZATION_ENABLED=true` separately enables target-selection confirmation, legal-angle branch retention, the 0.15-degree accumulated goal deadband, and brief revalidation of a previous shot. This is independent of the smoothing flag.
- `Turret.PREVIOUS_SOLUTION_MAX_AGE_SEC=0.10`: an old candidate is reused only after current trajectory validation; reuse does not renew its age. If no shot is valid, the turret holds its previous robot-relative goal. A held goal does not imply a valid shot.

Observe `Turret/Smoothing/CorrectionDeg`, `Turret/Stabilization/Status`, raw/commanded goals, and goal/profile/measured diagnostics. General NetworkTables logging is disabled; live dashboard visibility does not establish saved-log coverage.

## Feed policy

- `ShotMap.FEED_INTERLOCK_ENABLED=true`: normal feeding initially checks the readiness snapshot and current motion.
- `ShotMap.FEED_REQUIRE_TURRET_ALIGNMENT=false`: angular alignment does not block feeding. The turret still tracks, its travel limits remain enforced, and `Turret/AtAimGoal` reports the actual 4-degree tolerance check.
- `ShotMap.CONTINUE_FEED_AFTER_INITIAL_READY=true`: after automatic staging completes, speed/alignment/solution/motion readiness fluctuations do not stop sustained feeding. Disable, disabled mechanisms, inactive auto aim, illegal turret goals, and invalid bounded setpoints still stop it. Release/repress resets the sequencer.
- Initial staging starts the outtake after 50 ms and the spindexer after another 25 ms. Recovery after a hard stop requires readiness again. Setting the continuation flag false restores continuous readiness gating and recovery staging.
- With the interlock false, enabled operation is sufficient for staged feed; local subsystem guards remain active.
- Hold right trigger plus right-stick click for immediate manual force feed. With the interlock enabled, robot enable, enabled shooter roller, and finite bounded actuator commands remain required. Manual override alone does not latch automatic continuation.
- Hub shots use reduced teleop drive limits. Passback uses normal drive limits and bypasses shooting-specific measured speed/acceleration gates; motion freshness and trajectory validity still matter for initial automatic readiness.
- Holding A commands hood zero and blocks normal feeding through the bounded-setpoint guard. Release restores the latest normal hood target.

`Feed/Reason`, `Feed/ReadinessWarnings`, `Feed/MotionBlockers`, and `Turret/RobustShotReason` distinguish actual feed permission from diagnostic ShotReady changes. Turret PID and Motion Magic settings are separate from these policies.

# 06 — Measure moving-target tracking before changing the controller

## Prerequisites and outcome

Packets 01–05; stationary mechanisms qualified and calibration available. Outcome is a tested controller choice and documented motion envelope, not a blanket switch away from Motion Magic.

## Human inputs

Stationary control reports, allowed turret motion/limits, measured gearing, qualified shot distance range, and unloaded translation/rotation logs. Use existing Motor/Turret status signals rather than adding sensors. Missing moving data permits instrumentation and a disabled experiment implementation, not a claim that a controller upgrade is needed.

## Interfaces

Extend desired aim state with target angle, continuous unwrapped legal angle, optional target rate/acceleration, sample timestamp and validity. Clearly distinguish present physical state, predicted release state, and controller target horizon. A single rate term must not compensate the same latency twice.

## Implementation sequence

1. Log actual goal motion, Motion Magic reference, measured motion, feed readiness and chassis state in one time domain. Report profile tracking versus final-goal lag using packet 03 metrics and sample validity.
2. Evaluate the angular rate required by field-relative aim as both relative translation and chassis yaw change. Use a consistent frame, turret pivot offset and legal unwrap. Filter any numerically differentiated commands; invalidate derivatives across target changes, wrap discontinuities, long gaps or reference resets.
3. Keep the measured Motion Magic baseline as default. If valid runs demonstrate low profile-following error but excessive goal lag, add a compile/config-selectable experimental tracking mode using a supported Phoenix position/velocity reference with appropriate feedforward and bounded acceleration. Verify the installed library's units, features and license requirements before implementation. Do not use library names from a newer API without compiling against this repository.
4. Keep large reacquisition/stow moves on bounded profiles and use explicit bumpless mode transitions. Never activate two competing controllers, remove soft limits, or let rate compensation cross legal travel boundaries.
5. Preserve reference/configuration checks and handle unavailable aim rate by a named conservative behavior, not NaN. Do not conceal stale target samples by extrapolating indefinitely.
6. Provide an A/B trial routine/analysis using the same commands, currents, hardware and criteria. No model candidate becomes default solely on simulator results. If baseline meets criteria, retain it and document that the alternative is unnecessary.
7. Measure release timing from supplied video. Distinguish feeder request-to-first-exit, continuous stream timing and sensor/control latency. Evaluate sensitivity to plausible timing error before changing the model's fixed horizon; if no reliable measurement exists, label the parameter provisional.
8. Report tested speed, acceleration and yaw ranges per distance, including stationary fallback. Do not impose a new automatic drive limiter in this packet; packet 08 owns reviewed power profiles.

## Tests

Frames/signs, approaching/receding/lateral/rotational motion, target-zone transitions, angular wrap, legal-limit routing, derivative spikes, stale sample, controller-mode switch and reference reset. Bench-test deadline and output bounds. Measure solver/command-update timing on roboRIO through stage 10.

## Exit

A comparison report or clear evidence-needed status, exact control-mode selector/rollback, and qualified envelope entries. Return to [stage 6](../HUMAN-GUIDE.md#6-separate-moving-aim-error-from-projectile-error).

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

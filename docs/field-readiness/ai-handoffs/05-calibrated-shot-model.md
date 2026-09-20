# 05 — Calibrated hybrid shot model and bounded map representation

## Prerequisites and outcome

Packets 01–04. Deliver a reproducible calibration/model pipeline and an explicitly selected, versioned robot/browser package. A software pipeline may be finished before data arrive; a calibrated production model may not.

## Required evidence

Raw annotated launch records, repeat groups, a held-out session/operating-condition split, mechanical ranges, hardware ID and representative uncertainty. Consult [human stage 5](../HUMAN-GUIDE.md#5-use-data-to-choose-the-shot-model). If only net impacts exist, scoring confidence and Hub-clearance validation are unavailable. Preserve that distinction in the output.

## Starting architecture

`LinearShooterCalibration`, `RobustShotModel` conversion helpers, `GeneratedShotMapCoefficients`, calculator `trajectory.ts`, `evaluator.ts`, `polynomial.ts`, and `cli/generate.ts` duplicate assumptions. Current runtime selection is a generated distance/radial-velocity polynomial with lateral compensation and final validation. A better calibration must be used consistently by all those paths, including the inverse conversion from RPS to predicted exit speed.

## Model contract

Introduce a versioned calibration artifact loaded at startup, never fetched from the network in control loops. Include hardware ID, units/sign conventions, source dataset hashes, measured domain, selected model type, calibration parameters/knots, validation metrics, model version and review status. Unknown/mismatched hardware or malformed content yields an explicit unavailable-calibration state. A manually selected reviewed fallback is allowed; silent reuse of another wheel's calibration is not.

Separate actuator-to-launch calibration, projectile trajectory model, shot candidate selection and runtime representation. Results include calibrated/predicted/unqualified domain status and command provenance. Keep heuristic ranking separate from measured uncertainty and geometric validity.

## Implementation sequence

1. Audit forward/inverse conversion duplication and route all production conversions through one shared contract per language. Preserve parity tests for signs, boundaries and interpolation knots.
2. Fit the existing linear mapping and piecewise-linear alternatives. Use grouped holdout validation, uncertainty and repeat scatter. Fit hood dependence only when coverage/holdout evidence supports it. Refuse inversion of a non-monotonic or underdetermined speed mapping rather than selecting an arbitrary root. Report under-covered domains.
3. Compare generated polynomial residuals against a bounded interpolated synthetic map. Do not confuse synthetic approximation improvements with real-world accuracy. Use offline generation and a fixed documented runtime interpolation policy; invalid neighboring cells must not be bridged silently. Revalidate the final trajectory at each request.
4. Separate observed corrections from synthetic samples. If field observations justify residual corrections, interpolate only inside their validated support, preserve actuator bounds and reflect corrections in the same forward prediction used by final feasibility checks.
5. For persistent full-flight residuals, compare optional fitted drag against bounded empirical correction offline. Do not copy another team's drag coefficient or fit drag from data that cannot identify it independently of exit-speed/angle error. Activate a more complex model only if holdout evidence exceeds noise and preserves runtime budget. Default remains the simpler measured baseline.
6. Keep missing calibration and out-of-domain requests explicit. Add qualification flags for stationary/moving/target type and voltage or hardware conditions only where measured; do not assert a confidence percentage from a heuristic margin.
7. Export one source artifact and generated language-specific assets with hashes. `generate` must be an explicit reviewed command; tests must not silently rewrite production coefficients. Provide a comparison/shadow-report mode without commanding both alternatives.
8. Revalidate both Hub and passback after calibration changes. Do not extrapolate a short-range net calibration into long passback and call it qualified. Maintain explicit legacy selection for reviewed comparison, not automatic rejected-shot fallback.

## Tests and completion

Cover corrupted/mismatched artifacts, signs, units, duplicate/non-monotonic knots, boundary/out-of-domain inputs, invalid interpolation neighbors, stale calibration ID and finite bounded output. Use independent analytical fixtures as well as cross-language parity. Report holdout prediction error, worst supported error, repeatability, domain and runtime cost. Do not promote measured coefficients without a human review record. Return humans to stationary validation in stage 5.

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

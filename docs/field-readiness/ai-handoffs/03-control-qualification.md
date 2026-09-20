# 03 — Automated control-quality verdicts with human-approved tuning

## Prerequisites and outcome

Packets 01–02. Upgrade metrics into repeatable supervised qualification for turret, hood and shooter. No automatic gain search or match-time gain changes. Read [common contract](README.md).

## Human inputs

Actual ratio/travel, legal reference, approved trial amplitudes and voltage/temperature envelope, and a versioned [criteria form](../templates/02-acceptance-criteria.md). Provisional criteria must be labeled and acknowledged before trials. Code can be built without measured gains; valid hardware qualification cannot.

## Existing code to inspect

`TurretDiagnostics`, `TurretSubsystem.createStepTrialCommand`, `applyTestTuningFromDashboard`, shooter final-target readiness, `ConfigurationHealth`, and their tests. Current first-settled time stays latched after subsequent excursions; `resetAll` leaves some old step results; metrics accept invalid samples. Existing commands are not time-bounded and do not restore the prior suppression state. Gains/profile can partially apply as a mixed configuration.

## Public concepts

- `QualificationCriteria`: mechanism, units, trial type, legal amplitude/speed, sample-age/gap limits, settling dwell/deadline, observation interval, error/overshoot bounds, allowed electrical envelope, criteria revision and approval state.
- `QualificationSample`: timestamped command, profile and feedback, validity/status, configuration identity and relevant electrical/reference state.
- `QualificationResult`: `PASS`, `FAIL`, `INVALID`, or `INCOMPLETE`; reasons; metrics; coverage; configuration/criteria identity. INVALID identifies unusable evidence, not poor gains. Missing mandatory criteria remain INCOMPLETE.

## Implementation sequence

1. Keep a pure evaluator and a separate Test-mode trial coordinator with injected clock. Add fixed-step and moving-reference trial types; compute steady-state bias as an interval mean, not an instantaneous endpoint value.
2. Fix full reset, finite/monotonic-time validation, gap handling and settling semantics. Preserve first-settle time as a named descriptive metric, but require continued compliance through the observation window for PASS. Test both directions and at least three repetitions per configured condition.
3. Make each trial own the mechanism, preserve and restore prior aim suppression/mode, have a maximum duration, and terminate on cancel/disable, reference loss or essential fault. Reject clipped/near-zero requested steps as invalid coverage. Never move automatically at startup.
4. Capture coherent profile/feedback samples and verify physical update rates/ages. Report observation cadence; do not advertise a 100 Hz control-quality trace from a 50 Hz sampling loop.
5. Separate voltage saturation using available bus voltage, authoritative limiter flags, suspected current limiting, soft-limit clipping and model/profile error. Do not diagnose current limiting solely from high motor voltage.
6. Apply gains/profile through explicit Test-mode action, validate units/bounds, snapshot previous configuration, check all apply results/readback, and restore or inhibit on partial failure. Freeze configuration for the full trial. Include feedforward settings and criteria revision in the trial hash.
7. Provide evidence-based recommendation categories: reference/mechanical check, profile too demanding, feedforward characterization, feedback overshoot/bias, or moving-profile lag. Do not output a guaranteed new kP from one symptom. Document a SysId-compatible supervised routine if the team chooses characterization; verify vendor units/controller period.
8. Add equivalent position qualification for hood and speed/recovery metrics for shooter. Qualifying the flywheel unloaded does not qualify disturbance recovery under Fuel flow.

## Tests and return

Synthetic traces must independently cover oscillation, bias, brief settling then departure, repeated timestamp, NaN, stale sample, clipped trial, profile-limited lag, supply sag, partial apply and restored suppression. Scheduler tests cover cancel/disable/timeout. Produce a saved report and exact dashboard workflow, then return to [stage 3](../HUMAN-GUIDE.md#3-make-mechanism-control-repeatable-without-a-field).

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

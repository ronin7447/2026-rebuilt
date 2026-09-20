# 10 — Integrated verification, field qualification, and release evidence

## Prerequisites and outcome

Packets 01–09 with individual completion reports. Deliver a coherent release candidate and an honest qualification report. This packet does not authorize deployment, commits or publishing unless the current user request does so.

## Evidence required from humans

Completed setup, criteria, control, launch, power and incident forms; raw evidence locations; hardware/calibration/configuration identities; and actual practice/field outcomes. Without them, finish software integration and a pending-gates report. Never fill unknown results with expected outcomes.

## Implementation/review sequence

1. Inspect the integrated diff and each packet report. Confirm shared contracts agree on identity, hard protection, reference validity, metadata units, model domain and logging. Verify all legacy/experimental selectors have documented defaults and do not accidentally enable experiments.
2. Check operator bindings and Sendable command lifecycle under the scheduler. Ensure test commands cannot run in teleop, maintenance cannot unintentionally re-zero during a match, override is momentary, and each command stops/cleans up correctly.
3. Run full Java tests, calculator tests/build, power-analysis tests and any new calibration/qualification tools. Write exact commands and distinguish cached tasks. Use `--rerun-tasks` for the final candidate if a fresh complete run is needed; never call generation commands as tests when they mutate production artifacts.
4. Run deterministic fixture/replay coverage for both alliances, Hub/passback, moving/stationary, invalid domains, reset/stale-data paths, failed logs, configuration/reference faults and mode changes. Compare independent physical/analytical expectations alongside cross-language parity.
5. Verify timing instrumentation includes successful, rejected and failed solve paths, total scheduler duration and recording overhead. Ensure new file parsing/model generation happens outside enabled periodic control. Runtime map/solver loops must be bounded.
6. Produce a release manifest: git revision/dirty state, hardware revision, calibration/criteria/data hashes, configuration/current limits, firmware assumptions, approved shot/motion/power envelope, controls and rollback artifact. Use repository-relative links and team evidence URIs so another computer can audit it.
7. Review human qualification against the predeclared criteria, not against whatever happened to work. The existing three consecutive 150-second combined-load gate remains necessary. Distinguish sensor-confirmed resets, observed DS brownouts and inferred events; missing logging is a coverage failure.
8. If only a net/partial target was used, mark scoring geometry, field localization/alliance/autonomous gates pending. Do not relabel net hit rate as Hub accuracy. A stationary-only qualified release may be an intentional fallback; document it explicitly.
9. Add a release checklist and concise match-day operator card linked from the human guide. Include requalification triggers: flywheel/compression/gear/reference/camera/firmware changes and major controller/model modifications.
10. Inspect old qualification docs for contradictions with the completed implementation. Update them to point to the current guide and correct obsolete claims. Keep the historical implementation report intact as history.

## Required report

List implemented/software-verified/practice-verified/field-qualified status separately, tests with results, actual evidence locations, remaining failures, selected defaults and next actions. Include known limitations and any experimental code still disabled. If brownouts, reset/reference loss or sustained overruns remain, the stability objective is not complete.

Return the team to [stage 10](../HUMAN-GUIDE.md#10-rehearse-a-complete-match-and-release-deliberately). No hardware tests can be certified by a desktop-only agent.

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

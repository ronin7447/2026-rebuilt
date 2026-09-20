# AI implementation handoffs — field readiness

These are **new, unimplemented work packets** created 2026-09-20. They extend the first implementation; do not confuse them with the earlier temporary handoff set. The team explicitly requires this set in the repository for multi-computer work, overriding the handoff skill's temporary-directory default.

Read the [human guide](../HUMAN-GUIDE.md), [existing implementation report](../../implementation/ultimate-sotm-implementation-report.md), and the relevant packet. Source paths inside packets are relative to the repository root. Start from your current checkout's actual files; the audited revision was `39eaf45b48c6d1a3eb74d06a11a73b3040e85166` on `ultimate-sotm` with no pre-existing tracked modifications at this documentation session's start.

## Common contract

- The team has not tested the previous implementation on the robot. The flywheel is changing. Existing motor-to-launch regression, 0.14 s latency, PDH ID 1 and other provisional hardware assumptions are not newly validated by this document.
- The team can launch into a practice target/net, but room dimensions and realistic Hub access are unknown. Collect them through the supplied setup form. No additional sensors; video/operator observation is the source of Fuel-departure/scoring labels.
- Keep the accepted stale-vision warning-only policy and deliberate manual override, while making hard reference/configuration/actuator protections explicit. No uncontrolled automatic gain search, field deployment, or adaptive power controller is part of these packets.
- Default to supervised automatic scoring/recommendations, with a human approving gain/configuration changes. This is a documented planning default for the unanswered preference, not a measured team hardware requirement.
- Read current code, AGENTS instructions if present, and current diffs before editing. Preserve unrelated work. Do not reset branches or blindly regenerate coefficients. Recheck APIs against the installed vendordeps, not documentation for an incompatible version.
- Do not implement an entire later packet just because you touch one of its files. Add the minimal interface needed, document it, and keep default behavior explicit. Packets share RobotContainer/Constants/subsystem files, so sequential execution is recommended.
- Requirements below describe implementation outcomes. Any dashboard/API name not specifically called existing is proposed and must be created/documented by its packet. Never tell a human to press a proposed control before it exists.
- When measurements are missing, implement the tool, schema, guards and synthetic tests; report human inputs and hardware qualification pending. Do not invent real gains, physical stops, success rates, sensor events or observed timing. Unknown values remain unknown.
- Work without adding unrelated dependencies or broad vendor updates. Match existing Java 17/GradleRIO/TypeScript conventions. Read and use installed primary vendor docs when introducing control APIs.
- Production runtime must be bounded: no regression fitting, file generation, heavy optimization, blocking configuration retry loops or disk synchronization in enabled periodic control.
- Run targeted tests and the affected complete suite. Report exact commands, failures and cached-versus-executed status. Simulated traces verify algorithms; they do not establish physical performance.
- Keep test artifacts/datasets small and labeled in git. Store raw videos/logs externally with portable team locations and checksums. Do not embed user-specific absolute checkout paths, temporary paths, credentials or private tokens in documentation.

## Packets and progress

Update status/evidence columns only when work is actually performed. Documentation delivery is not implementation completion.

| Packet | Depends on | Code status | Human evidence |
|---|---|---|---|
| [01 Control/reference integrity](01-control-and-reference-integrity.md) | Setup form | Not implemented | Pending |
| [02 Recording/preflight](02-session-recording-and-preflight.md) | 01 | Not implemented | Pending |
| [03 Control qualification](03-control-qualification.md) | 01–02 | Not implemented | Pending |
| [04 Calibration capture](04-calibration-capture.md) | 01–03 | Not implemented | New wheel measurements pending |
| [05 Calibrated shot model](05-calibrated-shot-model.md) | 01–04 + measured data for promotion | Not implemented | Pending |
| [06 Moving tracking](06-moving-turret-tracking.md) | 01–05 + baseline trials | Not implemented | Pending |
| [07 Feed recovery](07-feed-throughput-and-recovery.md) | 01–06 + burst evidence for behavior changes | Not implemented | Pending |
| [08 Power profiles](08-power-trials-and-profiles.md) | 01–07 + load evidence for promotion | Not implemented | Pending |
| [09 Pose/mode resilience](09-pose-and-mode-resilience.md) | 01–08 | Not implemented | Pending |
| [10 Integration/release](10-integration-and-release.md) | 01–09 | Not implemented | Pending |

Tools/schema portions can be built while waiting for human measurements. Promotion of fitted calibration, altered recovery/control strategies or new electrical limits depends on that evidence. Do not mark a packet hardware-complete merely because its code portion is complete.

## Suggested skills

Use `domain-modeling` only when modifying terms or consequential accepted decisions; read `.agents/skills/domain-modeling/SKILL.md` and its formats. Use `grilling` for a genuinely new consequential decision that cannot be answered by code or supplied records, not to reopen the accepted defaults automatically. Use `handoff` when transferring unfinished work; repository storage remains the explicit requirement for this package. `grill-with-docs` composes grilling/domain-modeling when both are needed. If a Skill tool is unavailable, read the local SKILL.md files directly.

For actual dataset analysis or spreadsheet deliverables, use the relevant available analysis/spreadsheet skill. Do not invoke them simply because robot code logs a number. Never spawn new tasks or agents solely because this index has multiple packets; follow the current user's authorization and applicable instructions.

## Completion convention

Use [the report form](../templates/07-agent-completion-and-issue.md). Include code revision, tests, changed interfaces, current default flags, evidence supplied, provisional values, hardware tests still needed and the next human step. Add any named run report as a link here. Do not duplicate raw observations or other specifications.

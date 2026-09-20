# 07 — Feed diagnostics, measured recovery, and reliable manual clearing

## Prerequisites and outcome

Packets 01–06. Keep continuous readiness and warning-only vision policy. Deliver explanatory feed diagnostics first; change recovery behavior only if measured evidence justifies it.

## Human inputs

Single-shot and burst logs/video, the calibration/hardware IDs, speed droop/recovery traces, blocker durations, and the desired throughput/accuracy criteria. No departure sensor exists: require labels for actual launch counts; otherwise report only actuator-on time and inferred recovery.

## Existing code

`ShootCommand`, `FeedSequencer`, `ShotReadinessSnapshot`, `RobotContainer` bindings, indexer/final-feed subsystems. Current re-entry includes stable readiness followed by staged feed delays. The interlock-disabled branch is not an exact return to historical behavior; document actual semantics and do not use it as a blanket remedy.

## Implementation sequence

1. Preserve the current sequence as baseline. Record per-blocker episode start/end/duration, readiness transition, commanded targets, feedback, override and staging phase. Handle overlapping blockers without double-counting total pause time.
2. Separate hard protection from soft readiness in typed state. Hard faults include disabled operation, invalid/unbounded commands, lost reference, failed essential configuration or critical invalid feedback. Manual override must not bypass these; old finite setpoints alone are insufficient proof of a usable actuator.
3. Analyze matched bursts with video: first departure, actual inter-departure times where observable, recovery time and miss pattern. Provide a report identifying which blockers dominate. Never label wheel dips as confirmed individual Fuel events without validation.
4. Only if data shows recurrent normal shot droop causing needless full restart, introduce explicitly configured entry/hold speed tolerances and a warm-recovery state. Human-approved bounds come from measured launch sensitivity and holdout accuracy, not a larger arbitrary tolerance. Hysteresis cannot conceal bad sensor age or a failing reference.
5. Preserve immediate hard-fault stop, deterministic disable/cancel, and the existing initial cold-start sequence. Recovery dwell/timing must be named, logged and covered by state tests. Retain the previous strategy through an explicit configuration option for comparison.
6. Verify packet 01 manual reverse/stop ownership under shoot and anti-jam scheduling. Provide a documented deliberate clearing action; default is stop/driver recovery, not blind timed automatic reversal.
7. Add advisory jam suspicion only when command, speed/current and persistence support it. Distinguish empty hopper/no observed Fuel from confirmed obstruction. Test sustained blocked motion through simulation/fakes before any controlled hardware test.
8. Keep autonomous and teleop consistent on hard protections, with no autonomous manual override. List all NamedCommands/constructors affected.

## Tests

Cold start, soft speed droop, recovery chatter, repeated bursts, simultaneous blockers, target changes, hard fault during warm recovery, cancel/disable, override press/release and calibration/reference invalidation. Test command scheduler ownership, not only the pure sequencer. Use fake time and explicitly labeled synthetic traces.

## Exit

Blocker/throughput report; a justified state change or a finding that baseline should remain; revised operator clearing instructions; and measured criteria still requiring field confirmation. Return to [stage 7](../HUMAN-GUIDE.md#7-make-sustained-feed-predictable).

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

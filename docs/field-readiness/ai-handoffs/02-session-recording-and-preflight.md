# 02 — Portable session identity, explicit diagnostics, and preflight evidence

## Prerequisites and outcome

Packet 01; use existing `PowerHealthRecorder` and Phoenix logging. Deliver identifiable, reviewable records on any team computer without changing control behavior. See [common contract](README.md).

## Human inputs

Completed hardware/setup inventory, PDH ID/channel map, storage availability and a sample retrieved log when available. No sample log is required to implement the logger; real persistence and hardware communication remain unverified until the human smoke test.

## Starting evidence

`DataLogManager.logNetworkTables(false)` means most dashboard diagnostics and full feed blockers are not automatically in `.wpilog`. Current `BatteryLabel` is a constant `UNKNOWN`. `WpiDataLogSink.available()` and the PDH adapter's construction-success flag do not prove durable logging or current communication. `LastWriteSec` is not a flush acknowledgement.

## Interfaces and behavior

- Add immutable `TestSession` metadata: schema version, session/run ID, test type, operator label, battery ID, hardware revision, code revision/dirty indicator, configuration ID and calibration ID/hash when available. Paths/URIs to raw evidence are human metadata, never credentials.
- Publish editable disabled/Test-mode staging fields under `TestSession/`; apply via `TestSession/Apply`. Freeze an immutable identity at run start. Mid-run requested edits start a new segment only through an explicit action; they must not rewrite history.
- Add `TestSession/Mark` and an operator marker label. Log command/request timestamp separately from observed video event timestamps and their synchronization uncertainty.
- Add explicit records for turret legal goal, profile reference/rate, measured angle/rate, sample ages/status, configuration, trial verdicts, feed blocker list/transitions, homing outcome and reference state. Reuse the existing sampling infrastructure instead of another uncontrolled periodic notifier.
- Keep source timestamps/age versus recorder timestamps distinct. Log synchronized bundles at a documented cadence; mark asynchronous or stale samples. Do not lower odometry/control feedback rates.

## Implementation sequence

1. Read the existing recorder and signal dictionary, then enumerate exact additional signals and units. Separate slow identity/state events from high-rate numerical streams.
2. Bind typed state/snapshot suppliers directly; avoid using dashboard keys as authoritative control state or as an undocumented internal API.
3. Improve logger/PDH health reporting using supported APIs, staleness and retrieval checks. Expose initialization, observed activity, destination if known and reported write errors as separate facts. Never claim disk durability when the API provides no confirmation.
4. Record file/storage limitations and prevent logger exceptions from escaping robot control. Keep callbacks bounded and deterministic; storage/metadata work must not wait on the scheduler.
5. Add a preflight report for configuration/reference/session/calibration/PDH status. Unknown calibration is expected before packet 05; report it as unknown rather than green.
6. Update the signal dictionary and export instructions using an actual or representative exporter header. Keep log archives outside git; use repository-relative paths for source/fixtures.

## Verification

Test immutable identity, invalid metadata, mid-run edits, missing/stale source signals, logger initialization/write errors, missing PDH and correct flag semantics. Verify diagnostic field names and units from the written sample using a log reader. Use fake sinks in unit tests; label that test evidence synthetic.

## Completion and human return

Provide a field/key dictionary, startup/stop/retrieval instructions, schema version, and limitations of persistence checks. The human must retrieve and inspect a short real run before calibration. Return to [stage 2](../HUMAN-GUIDE.md#2-prove-you-can-trust-the-logs).

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

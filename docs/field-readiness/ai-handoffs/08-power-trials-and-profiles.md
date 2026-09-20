# 08 — Evidence-driven current limits and fixed operating profiles

## Prerequisites and outcome

Packets 01–07 for representative combined loads. Preserve the accepted logging-first policy. This packet may add selectable fixed test profiles and improve analysis; automatic voltage-triggered load shedding is not authorized by this roadmap.

## Human inputs

Real labeled logs and [power trial records](../templates/05-power-trials.md), verified battery/PDH mapping, mechanism completion/recovery criteria, and baseline/candidate run identity. Missing real logs means implement/test tooling and report pending evidence, not choose new competition limits.

## Implementation sequence

1. Audit `tools/power-analysis` against actual exports and the current signal dictionary. Correct aliases, timestamp units and missing-value handling. Boolean missing is unknown, never false evidence of no brownout.
2. Preserve source validity/age through joins and report data coverage. Reject duplicate/ambiguous identities. Fix event analysis that spans missing data or target changes; an unrecovered speed episode is censored, not a successful recovery. Separate movement detection from confirmed successful intake completion.
3. Expose analysis settings through a documented configuration file/CLI: voltage-event threshold, gap tolerance, current limits and shooter recovery tolerance. Include actual settings in output. The default plotted voltage dip is not a hardware brownout threshold or an acceptance margin.
4. Report total PDH current separately from summed motor supply current. Show limits/readback, stator torque context, current-limit evidence, voltage minima/duration, thermal trend, feed duty/recovery and loop health. Mark sum coverage/uninstrumented loads.
5. Add baseline/candidate comparison with battery/configuration matching and regression summaries. Include reverse-order or baseline/candidate/baseline runs in the suggested test plan to reveal discharge/temperature drift. No statistical certainty from one test.
6. If requested by the test lead after baseline evidence, add a default-off fixed acceleration-cap profile for teleop shooting tests. Preserve existing speed caps, steering behavior and meaningful driver control. Use coherent vector acceleration limiting with reset on disable/mode transition; report active profile. Do not modify autonomous behavior unintentionally.
7. For current-limit trials, keep values in reviewed named configuration artifacts/constants. Change them while disabled through a checked application pathway, never repeated configurator writes in enabled periodic control. Preserve X60/X44 and mechanism-specific constraints; never apply one universal limit.
8. Require human-reviewed evidence before promoting a candidate. Log all requested/applied values and profile identity. Provide a one-step explicit rollback to the last verified profile for the same hardware; do not weaken essential protections.

## Tests and completion

Test missing current/voltage/brownout state, unit scaling, duplicate timestamps, signal gaps, changed targets, incomplete recovery and channel mislabeling. Test acceleration limiter direction changes, combined axes, initial state, disabled reset and unchanged default path. Provide repeatable analysis commands and the real comparison report when data exist.

Return humans to [stage 8](../HUMAN-GUIDE.md#8-select-current-limits-from-combined-load-evidence). Do not invent a safe current total or claim a battery/cabling defect has been fixed in software.

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

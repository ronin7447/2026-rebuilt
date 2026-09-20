# Launch calibration and validation records

Keep raw data. The following fields describe the interchange required by packet 04; its importer/exporter will supply machine-readable CSV/JSON. Until it exists, use this form or a spreadsheet with these same names and explicit units. Leave missing observations blank/null, never zero.

## Session fields

`session_id`, `run_id`, date, `hardware_revision`, `flywheel_revision`, `code_revision`, `configuration_id`, `calibration_id`, `battery_id`, camera/video URI and checksum, frame time/PTS method, camera scale and plane, synchronization method/uncertainty, release height, target type/geometry, landing height, physical reference, Fuel condition, room limits.

## One row per launch

| Field | Value / notes |
|---|---|
| shot_id / run_id | |
| Observation method: direct near-exit / ballistic range-time / impact-only / unobserved | |
| Requested roller RPS / measured pre-departure RPS | |
| Requested hood position / measured position, units | |
| Measured hood physical setting and method if available | |
| Sensor status / source age at observation | |
| Robot pose, vx/vy/omega and frame if available | |
| Feed-request time / observed departure time and uncertainty | |
| Video departure frame / PTS | |
| Video first-impact frame / PTS | |
| Horizontal range from release point, m | |
| Release and landing heights, m | |
| Near-exit speed/direction estimate and uncertainty, if derived | |
| Target impact offsets and coordinates, m | |
| Outcome: make / miss / target-plane hit / unknown | |
| Rim/lip contact observed? | |
| Burst position / speed droop / recovery | |
| Fuel condition / visible spin note | |
| Raw data usable? Exclusion reason if not | |
| Fit or holdout group, assigned before fitting | |

Never count requested feed as confirmed launch. Do not delete misses. Unusable video may prevent velocity fitting while still providing a valid observed miss.

## Video trajectory annotations

Repeat this table for each shot; same-shot points are correlated, not independent trials.

| shot_id | frame index | presentation time, s | Fuel center x | Fuel center z | pixels or meters | scale/calibration ID | annotation uncertainty |
|---|---:|---:|---:|---:|---|---|---|
| | | | | | | | |

## Validation groups

| Condition/run | Kept out of fit? | Observed launches | Observed makes (real opening only) | Misses/unknown | Mean impact/dispersion | Hardware/battery change |
|---|---|---:|---:|---|---|---|
| | | | | | | |

Record net-only and limited-ceiling domains honestly. Record why a candidate is accepted/rejected and what additional measurements would distinguish competing models. Do not send only the winning regression coefficients to the agent.

# 04 — Capture reproducible shooter calibration observations

## Prerequisites and outcome

Packets 01–03. Implement measurement capture and offline annotation/import before fitting new production coefficients. The flywheel is changing; do not certify existing calibration for it. No new sensors or automatic video understanding are required.

## Human inputs

[Setup](../templates/01-setup-and-preflight.md) and [launch measurements](../templates/04-launch-measurements.md): space dimensions, release/target heights, legal hood/speed range, camera timing/scale, hardware revision and representative Fuel. If space dimensions are unavailable, build the tools but leave firing conditions unapproved.

## Data contract

Use versioned CSV/JSON interchange with explicit units and null for unknown measurements. Preserve raw observations separately from derived values. Each shot record needs session/run/shot ID, hardware/configuration/calibration identity, test mode, commanded and measured hood/roller state and sample age, robot pose/motion if available, release height, target/landing height, range, video URI/hash, frame/PTS annotations, observation method, uncertainty, outcome, and inclusion/exclusion reason. Group multiple observations of one launch by shot ID; they are not independent samples.

Distinguish `DIRECT_NEAR_EXIT_VIDEO`, `BALLISTIC_RANGE_TIME_ESTIMATE`, `TARGET_IMPACT_ONLY`, and `UNOBSERVED` methods. Distinguish an operator-requested feed from a video-observed departure. Store request-to-first-launch and burst inter-launch timing separately; the model's release prediction horizon is a separate quantity.

## Implementation sequence

1. Add supervised calibration mode that commands operator-selected bounded hood/RPS values, owns the relevant subsystems, suppresses field auto-aim, logs all actual states, and requires an explicit feed request. Do not automatically sweep speeds or fire Fuel on opening a dashboard.
2. Use the same hard reference/actuator protections as normal operation. Normal geometric Hub readiness may be inapplicable for a net test: make calibration authorization explicit, Test-mode only and visibly logged. Exiting restores prior mode and stops feeding. Never repurpose unrestricted force-feed as a hidden calibration mode.
3. Add start/end/mark shot controls and a session export. Design controls so missing beam-break sensing cannot fabricate shot counts or successful launches.
4. Supply an offline importer for manual video annotations in physical coordinates or pixel coordinates plus camera/scale calibration. Reject invalid time order, impossible scale and mixed units; propagate uncertainty. Do not assume nominal FPS equals video presentation time for variable-frame-rate footage.
5. Calculate near-exit position/time fits from multiple annotated points, with a clearly named approximation window. Produce estimated velocity vector/angle and residual diagnostics. Keep perspective limitations visible.
6. For range/time observations compute the vacuum-equivalent values described in the human guide, explicitly labeled as effective estimates. Never merge them silently with direct launch measurements.
7. Produce per-condition repeat scatter, excluded-video counts, requested versus measured state differences and data coverage. Preserve physical misses; video-invalid is not synonymous with a bad physical launch.
8. Export all data and configuration sufficient for packet 05. No generation of active robot coefficients in this packet.

## Tests

Import missing columns, null observations, unequal units, variable frame timing, invalid scale, duplicate IDs and early/late measurements. Use hand-calculated constant-velocity and ballistic trajectories plus noisy synthetic repeats. Verify input provenance survives export and no unobserved event receives a make/speed label.

## Exit

A runnable capture/import workflow, schema/sample fixtures and user instructions. Actual measurements remain human work. Return to [stage 4](../HUMAN-GUIDE.md#4-measure-the-new-flywheel-and-launch-behavior).

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

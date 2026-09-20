# 09 — Pose reacquisition, degraded-state visibility, and mode transitions

## Prerequisites and outcome

Packets 01–08. Improve localization diagnostics and transition integrity without changing the accepted warning-only policy for vision age.

## Human inputs

Measured camera transforms/calibration, surveyed stationary poses if available, occlusion/reacquisition logs, known reference-reset events and intended autonomous/teleop transitions. No local full field means field-coordinate and alliance tests stay pending; simulation does not fill that gap.

## Existing behavior

`VisionSubsystem` filters timestamps, on-field bounds, pose roll/pitch/Z and single-tag ambiguity. It does not currently assess innovation against the drivetrain pose at measurement time. Accepted-vision age is warning-only for feeding; keep that deliberate behavior. Review `VisionTimestampPolicy`, drivetrain historical pose APIs and `AutoAimTurretCommand` sampling reset logic.

## Implementation sequence

1. Log per-camera result identity/time, age, tags/ambiguity, estimated pose, selected measurement uncertainty and rejection reason. Validate numerical finiteness and temporal ordering per source; avoid dropping valid simultaneous measurements solely because another camera was processed first.
2. Compare candidate vision pose with the historical drivetrain estimate at capture time, not the current pose. Record translation/heading residual and whether history is available. A missing history sample is a distinct condition; do not substitute an unmarked current pose.
3. Implement a default-observe-only innovation evaluator. Derive candidate rejection thresholds from recorded data and supplied uncertainty. Keep sustained odometry drift/reacquisition as an explicit recovery scenario: a hard small residual cutoff can reject all correct vision after a shove.
4. If valid data supports gating, add a reviewed policy with a consistent-multiframe/multitag reacquisition path and explicit reset/transition behavior. Evaluate it on replay before activation. Do not automatically teleport/reset drivetrain pose from one contradictory image.
5. Keep stale vision as a warning and record calibration/model/pose status separately. A warning is not a covariance estimate or permission to bypass reference/actuator protection. Never make force-accept vision the production recovery solution.
6. Test and clear stale aim/readiness state across HOLD/PARK, trench/hood mode, suppressed aiming, unknown alliance, target-zone change, disable/re-enable, autonomous/teleop change and gyro/reference events. Reset derivatives/acceleration history where needed.
7. Surface an operator-readable reason and recommended recovery category, without adding automatic navigation or new sensing. Ensure dashboard alerts are rate-limited rather than flooding DS errors.

## Tests

Old/future timestamps, duplicate and out-of-order per-camera results, invalid numeric pose, large innovation caused by real motion versus time mismatch, history unavailable, two cameras disagreeing, prolonged occlusion with drift, consistent reacquisition and mode transitions. Tests must show blocked tags alone do not become a feed veto. Preserve red/blue coordinate parity.

## Exit

Replay/diagnostic report, observed thresholds with provenance, clearly selected observe-only or active policy, and completed transition tests. Return to [stage 9](../HUMAN-GUIDE.md#9-keep-pose-and-operating-modes-trustworthy). Keep unresolved field gates visibly pending.

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

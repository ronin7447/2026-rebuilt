# 01 — Control ownership, mechanical references, and bounded maintenance

## Prerequisites and outcome

Start here after the human completes stage 0. Read the [common contract](README.md). Deliver corrected control behavior and explicit reference validity before unloaded robot bring-up. This packet does not tune PID, change current limits, or qualify a firing range.

## Inputs needed from humans

Actual motor/gearing information; turret/hood reference procedure; mechanical travel limits distinct from calibrated shooting limits; intake homing travel; and intended reverse-release behavior. Use [setup form](../templates/01-setup-and-preflight.md). If mechanical limits/reference procedures are unknown, implement configurable validation and an unreferenced/inhibited state. Do not invent stops or use the shooter calibration interval as the complete hood travel range.

## Starting evidence and code

- `RobotContainer`: the Start sequence calls `.andThen(() -> Commands.waitSeconds(2))`, discarding the command instead of waiting. Start and POV-right can re-zero during ordinary operation.
- Intake left-trigger reverse conflicts with its onTrue disable action and periodic positive roller demand.
- `ZeroIntake` exists but is not exposed through an operator command. Stall detection ignores signal status/age; constructor resets do not constitute successful homing.
- Turret/hood `setPosition` status is ignored. Motor reset diagnostics exist but do not establish mechanism-reference validity.
- Hood accepts any finite target; the actuator needs measured mechanical limits, including any valid zero/stow position outside the firing range.
- Intake profile selection applies/retries configuration on the scheduler path.

## Implementation sequence

1. Add a small mechanism-reference state (`UNREFERENCED`, `REFERENCED`, `FAULTED`) with reason, timestamp and reset generation. Preserve a physically documented startup procedure only when explicitly confirmed by the operator; a constructor assigning zero is not automatic physical evidence. An essential reset or unsuccessful zero invalidates the affected reference until intentional recovery.
2. Ensure reference/configuration/enable health participates in hard actuator protection and feed override safety. Do not merely add dashboard warnings. Verify turret position-mode state in readiness. Keep existing status/age checks for shooter, hood and turret feedback. Permit unreferenced motion only through an explicit bounded supervised Test-mode homing/low-demand maintenance jog, with essential configuration, valid feedback, output bounds and timeout protections intact. This exception exists to establish direction/reference; it cannot enable normal position control or feeding and must stop on release, cancel or disable.
3. Replace unrestricted zero bindings with named supervised maintenance actions restricted to Test mode. Preserve ordinary emergency disable. Maintenance actions own their subsystems, stop outputs before zeroing, check the result and restore operation only on success. Compose any intended delay as an actual command. Publish the revised control map.
4. Expose `Maintenance/HomeIntake` as a Test-mode Sendable command with explicit cancel behavior. Reset stall dwell at entry and on invalid feedback; timeout/interruption never authorizes a new reference. Track both extension motors' configuration and follower availability.
5. Make roller target/direction explicit subsystem state. A held reverse command sets the negative target, owns the subsystem as appropriate, and stops on release/interruption. Default release behavior is STOP; resumption requires a subsequent operator intake command. Remove conflicting onTrue actions and periodic constant overwrites. Test interactions with shoot/anti-jam commands.
6. Validate hood commands against independently supplied mechanical limits before output and configure supported controller limits consistently. Preserve the narrower calibrated firing envelope as a separate readiness/model constraint. Out-of-range commands are rejected with reason rather than silently moving to an arbitrary clamp.
7. Remove repeated blocking profile configuration from runtime direction changes. Inspect installed Phoenix 26.1.2 capabilities and license requirements: prefer supported per-request constraints; otherwise use a preconfigured measured common profile. Do not prescribe an unsupported licensed request. If different extend/retract dynamics cannot be preserved without blocking, report the tradeoff and keep the last verified configuration until reviewed.

## Required tests

Use scheduler/command-level tests, not just isolated Boolean helpers: reverse hold/release/interruption; real two-second command sequencing with fake time; zero success/failure; initial unreferenced state and its bounded maintenance-only exception; controller reset; override unable to bypass hard health; stale stall signals; homing timeout/retry; legal stow outside firing envelope; illegal hood motion rejection; competing anti-jam commands. Simulate failed configuration without depending on real CAN hardware.

## Completion and human return

Provide tests, exact dashboard action names, before/after driver controls, unknown mechanical parameters, and any migration from old zeroing behavior. Return humans to [stage 1](../HUMAN-GUIDE.md#1-fix-control-and-reference-integrity-first). No hardware value is certified by this code change.

## Suggested skills and handover

Follow the [shared skill/implementation instructions](README.md#suggested-skills). Before finishing, fill the [completion and issue template](../templates/07-agent-completion-and-issue.md), update the packet status in the index, and identify the exact next human action. Link existing artifacts instead of copying their contents into a second specification.

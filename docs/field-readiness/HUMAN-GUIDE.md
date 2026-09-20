# Team 7447: from untested code to a competition-ready robot

**Working guide, 2026-09-20.** Start here. Complete the stages in order; record results instead of assuming a stage passes. This is a procedure and an implementation roadmap, not a claim that the robot is qualified.

The inspected branch is `ultimate-sotm`, revision `39eaf45b48c6d1a3eb74d06a11a73b3040e85166`. The previous improvements exist in code, but the team reports that they have not been tested on the robot. The flywheel is changing, so the previous exit-speed calibration is unqualified for the new hardware. No additional sensors are required by this guide.

## How to use this package

Assign a test lead to run the procedure, a driver/operator to control the robot, and a recorder to label runs and videos. One person may cover multiple roles when practical, but the driver should not simultaneously film a launch or edit gains. The people familiar with the mechanism set its actual travel and clearance limits.

At an **Implement** instruction, give the linked packet to an AI agent in this repository. Ask it to implement only that packet, run its specified tests, and produce a completion report. Do not ask it to implement all packets at once. Review the report, then perform the human tests below. An agent cannot replace physical measurements with a plausible number.

All guides, handoffs and forms live in this repository by explicit team request, overriding the handoff skill’s temporary-storage default. Links are relative so they work on any checkout. Keep raw logs/video in a shared team evidence location and record its URI/path and checksums in the forms; do not commit large recordings or credentials. The [packet index](ai-handoffs/README.md) explains dependencies and handover requirements.

### Starting an AI implementation session

Use a fresh task in this repository and send a prompt like:

```text
Implement docs/field-readiness/ai-handoffs/01-control-and-reference-integrity.md.
Read its common contract and prerequisites first. Preserve the current working tree.
The completed setup record is at: [team evidence location].
Implement this packet only, run its required software tests, and fill the completion
report template. List any missing physical measurements and the next human test.
Do not invent hardware results or deploy to the robot.
```

Replace the packet path for each stage and include the preceding completion report plus the required measurements. If a packet requests data you do not have, say so: the agent can build its tooling but must leave evidence-dependent activation pending. After review, follow the team's normal commit/push process to carry the documentation, code, configuration and small evidence manifests to other computers; a local file edit alone does not synchronize them.

Use four statuses: **not implemented**, **software verified**, **practice verified**, and **field qualified**. An unavailable facility gives **not yet tested**, not a passing result. A net can establish launch consistency; it cannot establish Hub lip clearance or Hub scoring accuracy.

### Stage map

| Stage | Human work | AI packet | Exit evidence |
|---|---|---|---|
| 0 | Identify hardware, space, and success criteria | None | Completed setup and acceptance forms |
| 1 | Verify safe control ownership and references | [01](ai-handoffs/01-control-and-reference-integrity.md) | Bench control checklist passes |
| 2 | Prove records and logs are usable | [02](ai-handoffs/02-session-recording-and-preflight.md) | Readable, labeled smoke-test log |
| 3 | Characterize and qualify mechanism control | [03](ai-handoffs/03-control-qualification.md) | Valid repeatable controller trials |
| 4 | Measure the new shooter | [04](ai-handoffs/04-calibration-capture.md) | Repeated launch observations and holdout data |
| 5 | Choose and validate calibrated shot behavior | [05](ai-handoffs/05-calibrated-shot-model.md) | Calibration report and accepted operating envelope |
| 6 | Validate aiming while moving | [06](ai-handoffs/06-moving-turret-tracking.md) | Tracking and moving-launch results |
| 7 | Validate sustained feeding and recovery | [07](ai-handoffs/07-feed-throughput-and-recovery.md) | Feed behavior and dropout report |
| 8 | Establish an electrical operating envelope | [08](ai-handoffs/08-power-trials-and-profiles.md) | Combined-load comparison and selected configuration |
| 9 | Validate localization and mode transitions | [09](ai-handoffs/09-pose-and-mode-resilience.md) | Occlusion/reacquisition and transition results |
| 10 | Rehearse, qualify, deploy, and maintain | [10](ai-handoffs/10-integration-and-release.md) | Completed release checklist and match-length evidence |

## 0. Prepare before enabling

1. Copy [setup](templates/01-setup-and-preflight.md) and [acceptance criteria](templates/02-acceptance-criteria.md) into a dated test folder outside the source tree. Keep raw logs/video there. Record the folder path in the run sheet.
2. Record the actual flywheel revision, wheel diameter/material/compression changes, motor type, gearing, and mechanical changes. Photograph or mark reference positions so later resets are reproducible. Do not assume a comment identifies the installed motor: the turret comment and the earlier hardware description differ.
3. Measure clear firing distance and usable height, including the whole predicted arc. Record target height and the shooter's release height above the same floor. Record whether the target has a real opening/lip or is only a net.
4. Inspect battery identity/condition, connectors, main power path, mechanism freedom, and known pinch/travel limits with the responsible team members. Repair physical faults before increasing gains or current limits to overcome them.
5. Choose the first operating objective in the acceptance form: useful near/mid distances, stationary success, allowable recovery time, and allowed drive speeds during shooting. Long-range and aggressive moving shots are later envelope extensions. Leave physically unknown values blank and schedule measurement; do not invent them to satisfy software validation.
6. Record revision with `git rev-parse HEAD` and inspect `git status --short`. Keep a known working software/configuration backup. The old flywheel's configuration is historical evidence, not a proven rollback calibration for the new wheel.

**Current software evidence:** the previous report recorded 56 Java, 28 calculator, and 5 power-analysis tests. During the latest investigation the Java Gradle task succeeded using up-to-date results and 28 calculator tests ran and passed. These do not prove robot behavior. The new packet agents must report their own commands and results.

**Default tuning policy:** score tests and recommend evidence-based changes; a person applies each candidate in supervised Test mode. No live automatic gain search or match-time learning is part of this guide. This resolves the unanswered tuning-preference question conservatively and can be revised explicitly later.

## 1. Fix control and reference integrity first

**Implement [packet 01](ai-handoffs/01-control-and-reference-integrity.md).** Do this before relying on the current dashboard trials or loading Fuel.

The audit found an intended two-second wait that is constructed but not scheduled, conflicting intake reverse behavior, unrestricted match-control re-zeroing, and incomplete handling of reference validity. Intake homing exists as a command but currently has no dedicated operator binding in RobotContainer. Packet 01 must provide and document a supervised homing action; do not look for an already-existing homing button.

### Human bench procedure

1. Follow the agent's updated control map. Identify disabled, Test-mode maintenance, normal driving, and shooting controls before enabling.
2. With no Fuel loaded, verify motor direction and position sign for each mechanism using packet 01’s bounded supervised Test-mode maintenance jog at low demand. This is the only motion exception, alongside homing, for an unreferenced mechanism; normal position commands and feeding stay inhibited. If a sign is wrong, stop and correct direction/units. Do not compensate by reversing gains.
3. Put turret and hood at their documented physical references while disabled. Perform the new supervised reference procedure. Confirm its success and legal target range. Do not rotate against a stop unless the procedure explicitly supports that mechanism.
4. Run intake homing once. Confirm it reports success only at the intended stop; then run a separate interrupted attempt and verify it stops without resetting the reference. Test timeout through the supported test/fake path or a supervised non-destructive setup, not by obstructing the mechanism with a hand.
5. Test intake forward, reverse, release, and interruption. Reverse must remain reverse while held; release must produce the documented stop/restore behavior. Test it while the other mechanism commands are active, unloaded.
6. Exercise disable and re-enable. A lost or unverified reference must be visible and must not be bypassed by force-feed. A disabled robot must not continue a test.
7. Confirm failed configuration or an invalid reference inhibits the affected mechanism. Inject failures in simulation first; hardware fault tests must follow the team's approved procedure without unplugging energized power wiring.

| Observation | Action |
|---|---|
| Zero succeeds in software but physical direction/angle is wrong | Correct reference, gearing, and signs; restart this stage. |
| Hood target is rejected | Distinguish mechanical travel limits from the calibrated firing range. Zero may be outside the firing range. |
| Homing times out or detects stale feedback | Leave unreferenced; inspect feedback and travel. Do not repeatedly force a zero. |
| A reverse or maintenance command is immediately overwritten | Return to packet 01; command ownership is still wrong. |

**Exit:** complete the mechanism section of the setup form. The operator can identify a valid reference and recover intentionally. No Fuel launches yet.

## 2. Prove you can trust the logs

**Implement [packet 02](ai-handoffs/02-session-recording-and-preflight.md).** Its session identity and explicit diagnostic records must work before time is spent collecting calibration.

1. Verify the REV PDH CAN ID and physical channel labels. The code's `1` is a provisional default. A channel number is not a motor CAN ID.
2. Set run ID, battery ID, hardware revision, configuration ID, and test type through the controls packet 02 supplies. Record the deployed git revision automatically where possible.
3. Start a short no-Fuel session. Move one known mechanism at a time and insert test markers. Disable, download the log, and open it before continuing.
4. Check existing indicators: `PowerLogger/Healthy`, `PowerLogger/Status`, `PowerLogger/PDHAvailable`, and `Health/Config/*/Applied`. Confirm PDH voltage is credible, the correct motor changes, units are known, and source age/validity accompanies readings.
5. Check that turret goal/profile/measured traces and feed reasons were explicitly recorded. The current recorder disables general NetworkTables logging, so a live dashboard number is not automatically a saved record.
6. Align a video marker with a logged event. Record the alignment uncertainty. If alignment is too coarse for individual launches, use it only for run-level comparison and annotate frames directly for launch timing.
7. Confirm the actual storage location. An available logger is not proof that every write succeeded or that a USB stick is being used. Keep a copy of the raw log and the original video.

### Existing power-analysis command

Export selected signals into a timestamped CSV. Use the current [power-analysis README](../../tools/power-analysis/README.md) and [signal dictionary](../implementation/power-signal-dictionary.md). From the repository root:

```sh
cd tools/power-analysis
npm test
npm run analyze -- /absolute/path/to/exported.csv /absolute/path/to/output BATTERY_ID CONFIGURATION_ID
```

The existing tool accepts CSV, not binary `.wpilog` or `.hoot`. Check the timestamp header and units when exporting; current timestamp aliases include `timestamp_sec` and `timestamp_ms`. Do not simply rename a microsecond timestamp to seconds. Inspect missing and duplicated rows before trusting durations.

The analyzer's default 7.0 V event threshold is a plotting trigger, **not a universal brownout threshold or qualification margin**. Packet 08 improves evidence/settings handling; the team's voltage margin is a separate acceptance criterion.

**If data are missing:** correct identity, recording, field export, device presence, or timestamps and repeat the smoke test. Do not fit a calibration to unidentified data or treat missing current as zero.

**Exit:** one short real log opens, carries correct identity, and shows coherent events. All following experiments get unique run IDs.

## 3. Make mechanism control repeatable without a field

**Implement [packet 03](ai-handoffs/03-control-qualification.md).** Keep Motion Magic as the initial turret controller. It supplies a profile; it does not certify that the mechanism follows well enough to score. See [CTRE's control explanation](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/device-specific/talonfx/motion-magic.html).

### First check the units and profile

The current configured turret cruise is 90 motor rotations/s with a 4:1 ratio: nominally 8,100 turret degrees/s. Acceleration similarly converts to 63,000 degrees/s². These are commanded profile constraints, not observed speed. Confirm the actual ratio and use a feasible measured profile under the existing current limits before judging PID.

1. Establish valid reference and unrestricted clearance for a small test. Reject a trial that clips a software limit; it is not an ordinary step response.
2. Start with small bidirectional steps, then the existing ±15° size. Repeat each direction three times using the same configuration and initial conditions. Let the mechanism stop between trials.
3. View final goal, Motion Magic reference, measured angle/velocity, output voltage relative to available supply, and actual current-limit indicators.
4. Save every repetition with a unique trial ID under its configuration ID. Change one gain or profile parameter at a time. Restore the previous candidate if a change is worse. Keep production constants unchanged until the selected candidate is reviewed.
5. Repeat with realistic cable orientation and warmed mechanisms. Test the hood and flywheel using their own units and appropriate position/speed trials. A shooter speed controller must also be tested under Fuel load later.

### Interpret the automatic verdict

| Result | Meaning and next step |
|---|---|
| INVALID | Stale/missing samples, changed goal/configuration, clipped motion, missing reference, or invalid test conditions. Repair the test and repeat; don't tune from it. |
| FAIL: profile tracking | The motor is not following the requested profile. Check feedforward, achievable acceleration, saturation/current constraints and mechanics before raising kP. |
| FAIL: final-goal lag with small profile error | The mechanism follows its profile, but the reference is too slow for the task. Review profile feasibility; moving-target tracking is stage 6. |
| FAIL: persistent overshoot/oscillation | Compare voltage/current and profile traces. Reduce demand or revise gains using the recorded evidence; don't automatically blame a single coefficient. |
| FAIL: steady bias | Recheck zero, friction/backlash and feedforward. Integral gain is not the first substitute for these checks. |
| PASS | This configuration passed this trial and criteria. It is not yet a scoring or all-conditions certificate. |

Use [control trial records](templates/03-control-trials.md). The acceptance form offers explicit provisional bench criteria; the dashboard must show their revision. A step that enters tolerance briefly and leaves again must not remain qualified. SysId can supply starting feedforward/feedback estimates, but those are starting points for measured validation, not guaranteed optimal gains. [WPILib guidance](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/system-identification/analyzing-gains.html)

**Exit:** valid repeatable stationary trials, approved settings, and documented profile limits. If mechanical faults or saturated profiles dominate, fix them before collecting projectile calibration.

## 4. Measure the new flywheel and launch behavior

**Implement [packet 04](ai-handoffs/04-calibration-capture.md).** Bring the [launch-measurement form](templates/04-launch-measurements.md), metersticks, camera/tripod, representative Fuel, and your practice target/net.

### Keep these two calibrations separate

- **Mechanism calibration:** motor feedback to physical hood setting and roller speed.
- **Launch calibration:** actual Fuel launch direction and exit speed as a function of measured mechanism state.

The hood's physical angle need not equal the departing Fuel's angle. Likewise, roller surface speed need not equal Fuel speed. The wheel change can alter slip, compression, spin and recovery. Reusing an old regression because its graph was linear is insufficient.

### Video setup

1. Fix the camera to the side, approximately perpendicular to the trajectory plane. Include scale markers in that same plane near the launch segment. Avoid perspective changes between runs; record camera settings and actual frame timing.
2. Prefer slow motion with sufficient light and a short exposure to distinguish the Fuel center over several frames. If the video lacks usable timing or scale, record the limitation rather than reporting an exact speed.
3. Measure release height, target/landing height, and horizontal distance from release point—not bumper or robot center. Mark where first impact occurs; rolling/bouncing distance is not flight range.
4. Annotate the last contact/first clear departure frame and at least several near-exit Fuel-center positions. Fit position versus time over a short visible flight segment; report uncertainty, not a single perfect value. Account for perspective and tracking quality before using the estimate.
5. Your existing landing distance/time method remains a useful independent observation. Under a no-drag approximation, horizontal velocity is range/time, and vertical velocity is `(landing_height - release_height + 0.5*g*time²)/time`. With drag these are **effective ballistic estimates**, not direct measurements of true initial velocity. Do not label them interchangeably in the dataset.

### Collection sequence

1. Begin at a low-demand, reachable launch configuration within verified travel and the space's arc/containment limits. Confirm it reaches the net before extending range or speed.
2. At one hood setting, gather five repeated single-Fuel launches at each of five distinct usable roller-speed setpoints. Pick setpoints within the safe test envelope; the guide does not prescribe full speed in an unknown room.
3. Repeat at low/mid/high usable hood settings if space permits. Full starting coverage is 3 hood settings × 5 speeds × 5 repeats = 75 launches. This is a collection target, not permission to fire outside the room's limits. If space supports fewer conditions, mark the smaller calibrated domain.
4. Use representative Fuel conditions and alternate groups so battery decline or wheel warming is not mistaken for a speed relationship. Record actual roller speed before departure and recovery after each launch, not just the requested setting.
5. Keep every observation, including misses and poor launches. Mark unusable video separately; never delete a bad physical outcome to improve the fit.
6. Reserve an entire later run or interleaved operating condition for validation. Neighboring frames from the same video are not independent holdout shots.
7. Measure a short burst separately. Single-Fuel calibration cannot establish sustained-feed behavior.

**If slow-motion video is unavailable:** collect range/time, measured mechanism states and repeated impacts, label the estimates appropriately, and avoid claiming direct exit-speed accuracy. **If the room cannot contain the full arc:** choose a lower contained trajectory or capture a short near-exit segment into a suitable net; do not intentionally strike the ceiling. Short-segment captures do not provide full-flight validation. **If only a net is available:** record plane/impact dispersion; mark Hub makes and lip clearance unobserved.

**Exit:** raw videos, run metadata, repeated observations, and a separate validation group. Give all of them to the next agent; do not send only regression coefficients.

## 5. Use data to choose the shot model

**Implement [packet 05](ai-handoffs/05-calibrated-shot-model.md)** and provide the stage 4 records plus the measured legal ranges.

Our default is a calibrated physical baseline with bounded empirical calibration/interpolation. Do not choose a lookup table merely because another team uses one. Three different things are often called a lookup table:

| Representation | What it represents | Main limitation |
|---|---|---|
| Motor-to-launch calibration | Measured actuator state to Fuel launch behavior | Changes with wheel/mechanism configuration |
| Measured scoring map | Successful hood/RPS settings versus distance | Needs representative target tests; does not by itself cover robot motion |
| Physics-generated map | Solutions computed offline across operating states | Inherits physical/modeling errors even with perfect interpolation |

Team 5987 describes generating model solutions and fitting experimental motor-to-launch behavior. Their measurements and coefficients are not transferable to this robot. [Primary team account](https://www.chiefdelphi.com/t/a-model-based-approach-to-shooting-on-the-move-team-5987-galaxia/518256)

### Review the agent's model report

1. Compare the current linear mapping with piecewise-linear measured calibration on held-out runs. Compare residuals versus speed, hood setting, hardware condition, and repeat scatter. Require an explanation of any multi-variable dependence rather than fitting a large surface to sparse data.
2. Compare the existing generated polynomial to denser interpolated physics samples separately. Its recorded maximum residuals of about 0.451 hood rotations and 7.78 RPS describe approximation of synthetic samples; they do not measure real shooting error. A better interpolator can reduce approximation error while leaving a wrong physical model wrong.
3. Keep the simpler calibration if the improvement is smaller than observation uncertainty or repeat scatter. Add a hood-dependent mapping only when holdout measurements support it.
4. For observed systematic flight error remaining after launch calibration, collect additional distance/flight data. Packet 05 may compare a bounded empirical residual correction or a fitted drag model offline; neither becomes active just because a more complex fit reduces training error.
5. Verify the chosen model's hardware ID, measurement domain, artifact checksum and holdout report. Require an explicit out-of-domain status. No silent extrapolation or borrowing the old wheel's calibration.
6. Use the same calibration/model package in the browser calculator and Java evaluator. A command correction must be reflected in the launch prediction used for feasibility; don't correct RPM and validate a different, uncorrected trajectory.

### Human validation

Shoot repeated stationary groups at near, middle, and far points **within the measured envelope**; include distances between calibration points. Record requested and measured settings, impact distribution, misses by direction, and clear versus contacting entry if the target supports it. Test both alliance transforms when a field reference is available.

If shots scatter widely at identical measured conditions, investigate Fuel consistency, feeding, wheel traction, hood play, and pose/reference errors. A higher-order polynomial cannot repair random mechanical variation. If mean impact moves systematically with distance, give that residual evidence to the calibration agent.

**Exit:** a reviewed calibration package that matches the new hardware, plus a stated qualified stationary envelope. A net-only result stays practice-qualified. Keep advanced moving shooting inactive until stage 6 passes.

## 6. Separate moving aim error from projectile error

**Implement [packet 06](ai-handoffs/06-moving-turret-tracking.md).** Start with unloaded tracking, then use Fuel only inside the room's verified path and firing envelope.

1. Track a fixed field target while the robot is stationary; then translate sideways slowly, approach/recede, rotate in place, and finally combine motions. Repeat both directions.
2. Record desired turret goal and goal rate, profile reference, measured angle/rate, current/voltage, chassis motion, and time alignment. Report final-goal and profile errors separately.
3. If both errors are large, return to stage 3. If profile error is small but moving-goal error is large, compare the measured Motion Magic baseline with the packet's optional bounded position-plus-velocity tracking candidate. Change no other tuning variable during the comparison.
4. Keep Motion Magic if the alternative does not produce a repeatable improvement under the same electrical limits. The experiment must preserve soft limits, reference validity, finite commands, and smooth mode transitions.
5. Add Fuel and compare stationary versus moving impacts. Measure launch latency by video where visible. Do not assume the code's 0.14 s prediction equals trigger-to-first-Fuel time, or that the first shot and a continuous burst have the same latency.
6. If the turret tracks but moving Fuel misses, investigate latency, robot/pivot velocity, localization, model bias, or release variability. Increasing kP will not directly fix those errors.
7. Record maximum demonstrated translation/rotation/acceleration conditions, not just top commanded speed. Extend the envelope one dimension at a time.

**No full field available:** use a surveyed fixed target in the practice coordinate frame; validate aiming and impact drift there. Do not rewrite competition coordinates to match the room. Mark alliance/field-target tests pending.

**Exit:** an accepted controller and a measured moving-shot envelope, or a deliberate stationary-shot operating mode for competition. A reliable stationary mode is a valid fallback if moving evidence is insufficient.

## 7. Make sustained feed predictable

**Implement [packet 07](ai-handoffs/07-feed-throughput-and-recovery.md).** Use single-Fuel, short-burst and sustained-feed tests in that order.

1. Review `Feed/Blockers`, `Feed/Permitted`, `Feed/OverrideActive` and logged transitions. Verify disable, trigger release and override release stop/restore behavior as specified.
2. Time spin-up, first observed departure, successive departures visible in video, speed droop and recovery. Without a departure sensor, use video/operator labels and call unobserved events unknown. Commanded feeder-on time is not Fuel throughput.
3. If every speed dip restarts staging, plot the blockers and measure time lost. The current sequence can add 0.10 s stable recovery + 0.25 s outtake delay + 0.10 s indexer delay after readiness recovers. This may be acceptable or may starve sustained feeding; measure it.
4. Packet 07 permits measured entry/hold hysteresis and a warm-recovery path only after the report demonstrates a problem. Hard faults still stop immediately. Do not solve starvation by disabling all readiness checks.
5. Exercise manual reverse/recovery. Confirm the operator can recover without a command fighting the reversal. Automatic reversing is outside the default scope without trustworthy evidence of a jam.
6. Evaluate a suspected jam with current, velocity, command state and video. An empty hopper and a jam can look similar without Fuel sensing; keep alerts advisory unless evidence supports a dependable distinction.

**If feeding reduces accuracy:** return to burst calibration/recovery, then adjust feed cadence or qualified envelope. **If nothing feeds:** inspect blockers and reference/configuration states before trying override. **If override is required routinely:** treat it as an unresolved defect, not a successful automatic mode.

**Exit:** sustained feed meets the predeclared throughput/accuracy target with acceptable recovery and clear driver recovery behavior.

## 8. Select current limits from combined-load evidence

**Implement [packet 08](ai-handoffs/08-power-trials-and-profiles.md).** Preserve current limits for the first baseline. Bring the [power trial form](templates/05-power-trials.md).

1. Verify battery condition, identity, charge state and connections. Run isolated drivetrain acceleration, steering, intake extension, intake roller, shooter spin-up and feeding tests.
2. Combine the loads progressively, then run a representative driver cycle. Record minimum voltage, DS brownout events, resets, total/supply currents, intake completion, shooter recovery, drive response, temperature and loop timing.
3. If an otherwise identical test changes drastically with battery or a connection, resolve that before software tuning. Voltage sag is not proof that a particular motor current limit is wrong.
4. Compare motor **supply** current for battery attribution. Use **stator** current to understand torque constraints. Do not sum stator current as battery demand. See [CTRE current-limit guidance](https://v6.docs.ctr-electronics.com/en/2025/docs/hardware-reference/talonfx/improving-performance-with-current-limits.html).
5. First candidate experiment: apply a fixed, reviewed drive-acceleration cap during combined-load shooting tests, preserving steering and intake-extension torque. The packet may implement a selectable fixed test profile, initially disabled. It must not add autonomous voltage-triggered load shedding.
6. If still needed, compare one supply limit, stator limit or motion profile at a time. Use baseline/candidate/baseline runs to reveal battery drift. Record applied limits, not just dashboard wishes. Retain a candidate only if voltage margin improves without failing mechanism or scoring criteria.
7. If extension stalls after a change, restore the last verified setting and examine torque, binding and acceleration. Do not keep lowering every limit until voltage looks good while the robot stops doing useful work.
8. Repeat on representative serviceable batteries. The selected configuration is a measured compromise; never label it a universally safe current budget.

**Exit:** a selected configuration with adequate voltage margin and successful useful work under combined loads. If brownouts remain, return to this stage; do not approve competition deployment based on isolated tests.

## 9. Keep pose and operating modes trustworthy

**Implement [packet 09](ai-handoffs/09-pose-and-mode-resilience.md).** The accepted policy remains: blocked/old AprilTags warn but do not alone inhibit feeding based on continuing fused odometry.

1. At surveyed stationary positions, compare the estimated pose to measurement. Test single-camera and multi-camera visibility where available.
2. Block tags briefly while stationary, then during controlled motion. Confirm the degraded-pose warning appears, odometry continues, and accepted-vision age is based on capture time.
3. Reveal tags again. Observe correction size and aiming changes. An old or inconsistent image must not silently cause a large aim jump. Packet 09 records residuals against historical pose and a recoverable rejection policy; it must not permanently lock out valid reacquisition after odometry drifts.
4. Test moving-target calculation while stopped/starting, changing target zones, autonomous-to-teleop transition, disable/re-enable, and supervised reset/re-reference procedures. Confirm no stale ready state survives a fault.
5. Verify trench/hood mode and aim suppression release through documented controls. Keep obstacle assumptions explicit; this guide does not introduce automatic obstacle navigation.
6. For defense-like disturbance, begin with non-contact motion and tag occlusion. If the team elects contact tests, use its established supervised practice procedure. Do not intentionally crash the robot to collect an error trace.

**If pose drifts during occlusion:** mark those shots degraded, reduce the qualified moving envelope or use the stationary fallback. Do not claim stale-vision warnings quantify position uncertainty. **If vision corrections jump:** investigate timestamp, transform, tag ambiguity, calibration and historical residuals; don't globally force-accept measurements.

**Exit:** logs explain occlusion/reacquisition and mode behavior, and drivers understand the degraded-pose/fallback indication. Field-specific behavior remains unqualified until tested with the actual geometry/targets.

## 10. Rehearse a complete match and release deliberately

**Implement [packet 10](ai-handoffs/10-integration-and-release.md).** Use the [release checklist](templates/06-release-and-incident.md).

1. Freeze a candidate: code revision, hardware revision, calibration hash, PID/profile settings, current limits, operating envelope and rollback package. Review all agent completion reports.
2. Run all software suites, including actual controller/command lifecycle tests introduced by the packets. The release agent must distinguish cached results, newly run tests, synthetic data and hardware evidence.
3. Rehearse the driver controls and fallback procedures unloaded, then with normal Fuel flow. Test autonomous separately before a full sequence.
4. Complete three consecutive 150-second combined-load trials across representative serviceable batteries. Require no DS brownouts, controller resets, failed intake movements or sustained loop overruns. Use scoring/throughput criteria written before the trial and record all attempts, not only successful groups.
5. Measure solver and scheduler durations on roboRIO while vision/logging/mechanisms run. Retain the prior target of less than 5 ms per solver evaluation with no attributable 20 ms scheduler overruns. Capture rejected and failed solves too, not only successful timings.
6. If only practice facilities are available, sign off the practiced envelope and mark realistic Hub, field localization, alliance and autonomous field gates pending. Use the first field opportunity for those specific tests rather than retuning everything at once.
7. Deploy the reviewed candidate through the team's normal process. Check revision/configuration identity on the robot, reference state, battery label, logger status, and controls before the match.
8. After a hardware, firmware, wheel-compression, gearing, reference or major software change, invalidate the affected qualification and repeat the relevant stages. A coefficient file existing on disk is not evidence it still matches the robot.

### Match-day quick procedure

Before each match: verify charged/identified battery and power connections; check mechanism references, configuration/calibration identity, logging and essential health; confirm selected autonomous and starting pose; brief the driver on the qualified shooting envelope.

After each match: download logs when practical, label match/battery, note resets/jams/misses/overrides and the conditions around them. Save an incident before changing settings. Fix one evidenced cause at a time.

If a new solver/controller is unstable, select the explicit reviewed fallback profile and record the change. A legacy solver switch cannot make the old wheel calibration valid on a different flywheel. If no fallback is qualified for the current hardware, finish the missing calibration/verification rather than presenting rollback as proven.

## What to send an AI agent after a failed test

Use [the issue/agent report template](templates/07-agent-completion-and-issue.md). Include packet number, revision, hardware/configuration/calibration IDs, exact reproduction steps, expected and observed behavior, whether the robot was enabled/Test/teleop, raw log path, video/run IDs, the criteria used, and the smallest relevant time interval. Tell it which measurements are unknown. Ask for a code-grounded diagnosis and a targeted fix, not a general rewrite.

## Scope boundaries

This roadmap does not promise a championship or infer performance from other teams' success. Its objective is a robot with a measured, repeatable operating envelope and understandable recovery behavior. It requires no new sensors, does not autonomously retune during matches, does not silently change the warning-only vision policy, and does not pick final PID gains or current limits without robot measurements.

The existing [implementation report](../implementation/ultimate-sotm-implementation-report.md), [accepted plan](../implementation/ultimate-sotm-plan.md), [glossary](../../CONTEXT.md), and ADRs remain the record of the first implementation. This guide adds the path from that untested code through further improvements and qualification.

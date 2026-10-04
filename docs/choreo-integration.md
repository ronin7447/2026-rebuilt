# Choreo with Team 7447's existing PathPlanner autonomous system

Inspected 2026-10-03 on branch `launch-speed-calibration`. This is an integration and test guide, not evidence of physical qualification. The team has not yet performed the field-readiness tutorial. The three supplied trajectories were inspected as data; they have not been deployed or driven as part of this review.

## Recommendation

Keep the working PathPlanner autonomous system. Use Choreo to generate experimental trajectories, then execute them through the existing PathPlannerLib AutoBuilder and chooser. No ChoreoLib dependency is needed for this route. Preserve a known-good PathPlanner auto as the competition fallback.

| Topic | PathPlanner | Choreo |
|---|---|---|
| Designing motion | Shape a Bézier path and specify rotations and constraints | Specify waypoints, constraints, and a robot model; an optimizer chooses motion and timing |
| Main benefit for this team | Familiar workflow with working robot integration | Potentially faster, coordinated translation/rotation for repeated collection cycles |
| Practical tradeoff | More manual path-shaping work | More dependence on realistic physical parameters and careful review of generated motion |
| Mechanism sequencing | GUI auto command groups and events already integrated here | Can use the same PathPlanner command system through interoperability; native ChoreoLib is a separate option |
| Unexpected starting state | Native PathPlanner paths can regenerate timing for the starting state | Imported Choreo motion is precomputed; begin at the intended pose, heading, and velocity |
| Reactive navigation | PathPlannerLib provides pathfinding APIs | Choreo generates planned trajectories; ChoreoLib adds routine/branching APIs, not automatic obstacle perception |
| Reliability | Depends on localization, configuration, margins, and tests | The same; an optimized trajectory is not a guarantee of better tracking or less current draw |

Both tools support swerve translation independently of chassis heading. Choreo is not required for shooting while moving. For this robot, the most useful first comparison holds the follower, PID gains, localization, and mechanism behavior constant and changes only the trajectory generator.

Sources: [PathPlanner path/auto editing](https://pathplanner.dev/gui-editing-paths-and-autos.html), [Choreo path editing](https://choreo.autos/usage/editing-paths/), [PathPlanner Choreo interoperability](https://pathplanner.dev/pplib-choreo-interop.html).

## What is already installed

- [PathPlannerLib vendor dependency](../vendordeps/PathplannerLib-2026.1.2.json): version `2026.1.2`.
- [RobotContainer](../src/main/java/frc/robot/RobotContainer.java): `configureAutonomousChooser()` supplies pose, odometry reset, measured chassis speeds, a velocity-mode CTRE output, PID controller, robot configuration, and a red-alliance flip supplier.
- `AutoBuilder.buildAutoChooser()` loads autonomous files. The dashboard key is `Auto Chooser`.
- `getAutonomousCommand()` returns the selected command.
- No `src/main/deploy/choreo` directory or ChoreoLib dependency was present at inspection.

PathPlannerLib 2026.1.2 accepts Choreo trajectory schema versions through 3; all three supplied files are version 3 with swerve samples. This is a source/schema compatibility check, not a robot runtime test. See the [version-pinned loader](https://raw.githubusercontent.com/mjansen4857/pathplanner/v2026.1.2/pathplannerlib/src/main/java/com/pathplanner/lib/path/PathPlannerPath.java).

## Findings in the supplied trajectories

The original files were supplied from Downloads and have not been copied into the deploy directory. Filename stems below omit `.traj`.

| File | Generated duration | Peak chassis speed | Peak translational acceleration | Final pose x, y, heading |
|---|---:|---:|---:|---|
| `LeftTrenchBump2Sweep_copy1` | 17.478 s | 5.102 m/s | 11.609 m/s² | 3.931 m, 7.386 m, 0.36° |
| `LeftTrenchBump2Sweep` | 17.424 s | 5.102 m/s | 11.611 m/s² | 0.370 m, 6.846 m, 90° |
| `LeftTrenchBump3Sweep_copy1_copy1` | 21.030 s | 5.103 m/s | 11.611 m/s² | 0.584 m, 5.943 m, 180° |

All start near `(4.354 m, 7.396 m, 0°)` at zero chassis velocity. All have `splits: [0]`, so there are no internal split sections. The editable parameters match the generation snapshots after expression values are normalized.

### Fix these before a full trajectory test

1. **Duration:** 2026 AUTO is 20 seconds. The three-sweep file cannot complete in AUTO, even without additional waits. The two-sweep files leave only about 2.5 seconds for any time added outside the trajectory. Recalculate the budget after lowering constraints; target a deliberate completion margin. See [official manual, section 6.4](https://firstfrc.blob.core.windows.net/frc2026/Manual/2026GameManual.pdf).
2. **Motion demand:** the samples exceed the repository's configured `kSpeedAt12Volts = 4.94 m/s`, and acceleration approaches 1.18 g. That does not prove the robot cannot follow them, but it leaves little tracking headroom and is an aggressive starting point given the brownout history. Use measured loaded performance. As an initial short-test setting only, consider 2 m/s and 2 m/s² on flat carpet, then increase based on measured error and voltage. Reduce further if needed; these are not certified limits for bumps.
3. **Bumper model mismatch:** the Choreo snapshot uses front/back half-lengths 0.34925 m and side half-width 0.3429 m: full length 0.6985 m, width 0.6858 m. [PathPlanner settings](../src/main/deploy/pathplanner/settings.json) use length 0.86 m, width 0.84 m. Measure the actual bumper envelope and account for deployed mechanisms, then reconcile both tools. Do not blindly assume either configuration is correct. Choreo requires the outside bumper dimensions: [robot configuration documentation](https://choreo.autos/document-settings/robot-configuration/).
4. **Wall clearance:** using the PathPlanner rectangular envelope and zero bumper offset, `LeftTrenchBump2Sweep` reaches approximately 7.7 cm beyond the x=0 field boundary around 14.50 s. Even its final pose places the envelope about 5 cm beyond that boundary. This is a conditional geometry finding, not a claim that the actual bumper has been measured. Correct the dimensions and move the path away from the wall with tracking margin.
5. **Trench/bump travel:** a field background does not validate clearance. Review the entire swept robot envelope, extended intake, turret/shooter height, and pitch over bumps. The listed constraints contain a field-boundary rectangle and lane constraints, but those alone do not protect against field structures. Neither a flat-ground trajectory nor a simple simulation validates loss of traction on a bump.
6. **Events:** every file has two markers named `EnableShooter` and one named `DisableShooter`, all with `event: null`. The robot has no bindings for those marker names. These files will not currently start or stop shooting.

## Step 1 — Create a small experiment

Keep the existing competition autos. In Choreo create a separate `ChoreoTaxiTest` trajectory: roughly 1–2 m straight travel in an unobstructed test area, constant heading, stop at both ends, conservative velocity/acceleration, and no mechanism events. Then create a separate short rotation test.

Do not begin by running an entire 17-second multi-cycle route. You do not need to finish every future improvement in the [human field-readiness guide](field-readiness/HUMAN-GUIDE.md) to inspect files or simulate this test. Before floor testing, verify drivetrain direction, pose/heading, the selected routine, and disable behavior. Shooting tests still require the relevant calibration/readiness work.

## Step 2 — Save the Choreo project into the repository

In the Choreo application, save the `.chor` project into:

```text
src/main/deploy/choreo/
```

Generate the trajectory and verify its `.traj` appears beside the project. Example future layout:

```text
src/main/deploy/choreo/Team7447.chor
src/main/deploy/choreo/ChoreoTaxiTest.traj
src/main/deploy/choreo/LeftTrenchBump2Sweep_copy1.traj
```

The filenames above are examples, not files created by this guide. Save the real `.chor` from your Choreo project; do not invent one from a trajectory snapshot. A `.traj` is sufficient for robot playback, but keeping the `.chor` and all associated `.traj` files together preserves project configuration for other computers. Commit those files along with the PathPlanner `.auto` wrappers. Do not use absolute Downloads paths in robot code.

Source: [Choreo saving documentation](https://choreo.autos/usage/saving/).

## Step 3 — Make a Choreo auto in PathPlanner

1. Open the same robot project in PathPlanner after generating the file in `deploy/choreo`. Refresh/reopen the project if the trajectory list is stale.
2. Use the GUI's Choreo-auto option and add `ChoreoTaxiTest` as a path command. Exact placement of the option depends on your GUI release; the generated file needs to be present for discovery.
3. Enable resetting odometry at the start. Physically place the robot at the matching start pose before running. Resetting odometry does not move the robot to that pose.
4. Save as `CHOREO_TEST_Taxi`. Confirm the `.auto` is under `src/main/deploy/pathplanner/autos`.
5. Inspect the saved wrapper: `choreoAuto` must be `true`, `resetOdom` must be `true`, and `pathName` must match the trajectory filename without `.traj`.
6. Build/deploy the test when ready, then select `CHOREO_TEST_Taxi` from the existing `Auto Chooser`. Merely copying a `.traj` does not add a chooser option: it needs an auto wrapper or an explicitly added command.

A minimal wrapper, for reference, is:

```json
{
  "version": "2025.0",
  "command": {
    "type": "sequential",
    "data": {
      "commands": [
        { "type": "path", "data": { "pathName": "ChoreoTaxiTest" } }
      ]
    }
  },
  "resetOdom": true,
  "folder": null,
  "choreoAuto": true
}
```

`2025.0` is the wrapper schema used by the installed 2026 library and existing autos, not a request to downgrade software. The wrapper is shown here rather than installed into deploy because the example trajectory does not yet exist.

The repository already has the needed Java integration for a basic driving-only Choreo auto. The alternative Java entry point is `PathPlannerPath.fromChoreoTrajectory(name)` followed by `AutoBuilder.followPath(path)`; a manually assembled auto must also handle its initial odometry reset and alliance behavior. Prefer the GUI wrapper for the first test.

Source: [PathPlanner interoperability documentation](https://pathplanner.dev/pplib-choreo-interop.html).

## Step 4 — Verify alliance and start-state behavior

Design a canonical trajectory on the blue side. The existing red-alliance supplier is already passed to AutoBuilder. Preview and simulate both alliances, checking starting position, chassis heading, trench choice, Hub side, and final pose. Do not manually flip a trajectory and then let the library flip it again.

Confirm the field layout and symmetry match REBUILT. An unknown alliance defaults to blue in the current code. Do not interpret that fallback as evidence that the correct alliance was received.

Choreo samples already contain their speed profile. Lowering PathPlanner GUI defaults does not retime an imported Choreo trajectory. Lower constraints in Choreo and regenerate. Do not slow it by multiplying drivetrain output: the trajectory clock and event timing would continue at the original rate.

## Step 5 — Connect mechanisms explicitly

The current registered command names are:

| Existing name | Meaning in this code |
|---|---|
| `EnableIntake` | Extend intake; turn on roller after successful extension |
| `DisableIntake` | Stop intake roller and request retraction |
| `EnableAutoAim` | Enable turret position mode/hood and clear auto-aim suppression |
| `DisableAutoAim` | Suppress automatic aiming |
| `ShootSequence` | Run the shooting/feed command until interrupted |
| `EndShootSequence` | Stop shooter, indexer, and outtake; conflicts with the active shooting command |

For PathPlanner interoperability, attach a **Named Command action** to each Choreo event marker. A marker label alone does not execute a registered command. An explicit EventTrigger binding is another coding option, but none exists here for the supplied marker names.

For example, the marker labeled `EnableShooter` can contain this command payload:

```json
"event": {
  "type": "named",
  "data": { "name": "ShootSequence" }
}
```

The label may remain `EnableShooter`; the command action is what matters. The stop marker should attach `EndShootSequence`. Add intake and auto-aim actions deliberately rather than assuming the supplied trajectories contain them. Regenerate after moving marker target waypoints.

First verify event scheduling using temporary logging actions without Fuel. Only enable real shooting after the relevant calibration and readiness checks. `ShootSequence` never finishes on its own: putting it directly in a sequential auto would prevent advancing to the next command. Use a bounded deadline/wait group for a stationary shooting interval, or path event actions with an explicit stop for moving shooting. Ensure stop/cleanup works when the path finishes, is interrupted, is disabled, and transitions to teleop. Recheck mechanism state after each case.

The intake extension command also has no built-in timeout and has interruption behavior that needs hardware qualification. Use bounded compositions for experimental autos and do not assume an unfinished extension automatically retracts on interruption.

Sources: [Choreo marker command bindings](https://choreo.autos/usage/editing-paths/#pathplanner-interop), [RobotContainer command registrations](../src/main/java/frc/robot/RobotContainer.java), [ShootCommand](../src/main/java/frc/robot/commands/compositions/ShootCommand.java), [ManipulateIntake](../src/main/java/frc/robot/commands/intake/ManipulateIntake.java).

## Step 6 — Decide whether to use splits

Your supplied files have no internal splits. If you want to arrive, stop, wait for shooting readiness, shoot for a bounded interval, and then continue, generate actual stopped segments or add both suitable stop constraints and split points in Choreo. Splitting alone does not make an arbitrary moving waypoint a safe stop.

Reset odometry at the start of the whole routine, not at each segment. Preserve continuity across segment boundaries. Add waiting/shooting time to the full AUTO budget. If retaining continuous moving-shooting sections, the current slower sections are reasonable places to experiment, but marker timing and successful shots still require robot measurements.

## Step 7 — Qualify in increasing scope

| Test | Record/check | If it fails |
|---|---|---|
| Load/simulate taxi | Chooser entry, no file/version errors, correct initial pose, both alliances | Fix file placement/wrapper/alliance configuration before driving |
| Short straight drive | Desired vs measured pose, final error, voltage, disable stops motion | Correct sign/units/reset first; then follower tuning or lower trajectory demand |
| Short rotation | Desired vs actual heading and translation drift | Check module/gyro configuration and rotation tracking |
| One collection segment | Clearance with real bumpers/intake, tracking error, current/voltage | Fix dimensions/path margins; regenerate at achievable constraints |
| Event-only verification | Each intended command starts and ends exactly once per marker | Fix bindings and command ownership before enabling feed |
| Calibrated shooting segment | Readiness/blocked reason, shots, pose quality, power | Diagnose shot calibration vs localization vs feed; do not hide failures by bypassing readiness |
| Full routine | End-to-end duration, endpoint, all mechanism cleanup, repeatability | Shorten or simplify the routine; retain the working fallback |

Use the existing `PathPlanner/Field` display for desired/actual path inspection, plus Driver Station logs and available power logging. Run repeated trials with representative Fuel load and healthy batteries at different states of charge. Simulation validates wiring and scheduling, not real traction, bump dynamics, shooting, or brownout resistance.

Keep an identifiable known-good auto and record the code commit, trajectory revision, settings, battery ID, and outcome for every comparison. Do not change shooter calibration, follower gains, and trajectory constraints simultaneously.

## Later coding improvements, after the basic integration works

1. **Module force feedforwards:** the current AutoBuilder output takes only `speeds`. Choreo files include per-module forces, but those additional trajectory force feedforwards are not forwarded to CTRE. The existing motor velocity loop still has its own configured gains/feedforward. A later change can use AutoBuilder's two-argument output and CTRE wheel-force inputs. Verify units, module order, robot-relative frame, and the installed APIs; compare tracking/current on the same path before and after. This is not required merely to load Choreo.
2. **Auto validation:** check every deployed wrapper/reference, event binding, duration budget, bumper envelope, start/end state, and finite sample values before deployment. Report load errors clearly and keep an explicit do-nothing fallback.
3. **Bounded mechanism actions:** improve intake timeout/interruption handling and auto end cleanup, with tests for cancellation and teleop transitions.
4. **Native ChoreoLib:** consider it only when reactive branching/trajectory triggers provide a concrete advantage over the working integration. It requires an appropriate 2026 dependency, a trajectory follower, chooser/routine integration, and tests. Current online Choreo docs may show 2027 alpha APIs; do not copy those dependencies into this 2026 project uncritically.

The prerequisite for faster autonomous operation is a measured, repeatable robot. Changing the trajectory editor alone does not complete the unperformed hardware qualification work.

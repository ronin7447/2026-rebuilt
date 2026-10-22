# Rōnin 7447 software technical binder

A responsive, single-page technical binder for public and judging conversations. Blue team branding, sentence-case typography, interactive release prediction, staged-feed exploration, technical expanders, and print styling.

## Run

From this directory:

```sh
npm run dev
```

Open http://127.0.0.1:7447. No package installation is required. `index.html` also works directly from disk; web fonts have local fallbacks.

## Build

```sh
npm run build
```

Deploy `dist/` to any static host. The Sites identity is retained in `.openai/hosting.json`.

## Content provenance

Content follows `ultimate-sotm` at `3323e73`. Public copy describes implemented software architecture and configured behavior. The interactive model illustrates horizontal compensation using the branch's 0.14 s release horizon and turret-pivot offset, with a fixed illustrative 0.8 s flight time. It is independent of the full robot solver. The feed explorer uses the branch's 50 ms + 25 ms staging configuration.

| Binder topic | Repository source |
| --- | --- |
| Team identity | `README.md` |
| Shared motion and release prediction | `src/main/java/frc/robot/util/sotm/ShotMotionEstimator.java`, `ReleaseStatePredictor.java` |
| Shot solver and 3D transforms | `src/main/java/frc/robot/util/sotm/RobustShotModel.java`, `ShooterFrameTransform.java` |
| Vision and timestamps | `src/main/java/frc/robot/subsystems/VisionSubsystem.java` |
| Readiness and staging | `src/main/java/frc/robot/util/sotm/ShotReadinessSnapshot.java`, `src/main/java/frc/robot/commands/compositions/FeedSequencer.java` |
| Flywheel hysteresis | `src/main/java/frc/robot/util/ShooterSpeedReadiness.java` |
| Drive shaping and driver controls | `src/main/java/frc/robot/RobotContainer.java`, `src/main/java/frc/robot/util/sotm/ShootingDriveLimiter.java` |
| Autopilot | `src/main/java/frc/robot/commands/autopilot/Autopilot.java` |
| Telemetry and analysis | `src/main/java/frc/robot/PowerHealthRecorder.java`, `tools/power-analysis/` |
| Shared calculator contract | `tools/shot-calculator/`, `shared/shot-evaluator-fixtures-v1.csv` |

## Editing

Update narrative in `index.html`, visual design in `styles.css`, and interactions in `app.js`. Source links are pinned to the source revision in `app.js`. No robot code is imported or modified. The diagrams are vector illustrations of the software concepts.

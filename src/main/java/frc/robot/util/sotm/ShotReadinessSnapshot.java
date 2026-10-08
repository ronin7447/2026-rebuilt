package frc.robot.util.sotm;

import java.util.ArrayList;
import java.util.List;

/** Immutable, non-circular explanation of whether feeding is currently permitted. */
public record ShotReadinessSnapshot(
    double timestampSec,
    boolean enabledOperation,
    boolean solutionValid,
    double solutionAgeSec,
    boolean shooterRollerEnabled,
    boolean shooterAtTarget,
    boolean hoodEnabled,
    boolean hoodAtTarget,
    boolean turretAutoAimActive,
    boolean turretAtGoal,
    boolean turretGoalLegal,
    boolean setpointsFiniteAndBounded,
    double acceptedVisionAgeSec,
    List<String> blockers
) {
    public ShotReadinessSnapshot {
        blockers = blockers == null ? List.of() : List.copyOf(blockers);
    }

    public static ShotReadinessSnapshot evaluate(
        double timestampSec,
        boolean enabledOperation,
        boolean solutionValid,
        double solutionAgeSec,
        boolean shooterRollerEnabled,
        boolean shooterAtTarget,
        boolean hoodEnabled,
        boolean hoodAtTarget,
        boolean turretAutoAimActive,
        boolean turretAtGoal,
        boolean turretGoalLegal,
        boolean setpointsFiniteAndBounded,
        double acceptedVisionAgeSec,
        double maxSolutionAgeSec
    ) {
        List<String> blockers = new ArrayList<>();
        if (!enabledOperation) blockers.add("robot-not-enabled");
        if (!solutionValid) blockers.add("invalid-shot-solution");
        if (!Double.isFinite(solutionAgeSec) || solutionAgeSec < 0.0 || solutionAgeSec > maxSolutionAgeSec) {
            blockers.add("stale-shot-solution");
        }
        if (!shooterRollerEnabled) blockers.add("shooter-disabled");
        if (!shooterAtTarget) blockers.add("shooter-speed");
        if (!hoodEnabled) blockers.add("hood-disabled");
        if (!hoodAtTarget) blockers.add("hood-position");
        if (!turretAutoAimActive) blockers.add("auto-aim-inactive");
        if (!turretAtGoal) blockers.add("turret-alignment");
        if (!turretGoalLegal) blockers.add("turret-software-limit");
        if (!setpointsFiniteAndBounded) blockers.add("invalid-setpoint");
        return new ShotReadinessSnapshot(
            timestampSec,
            enabledOperation,
            solutionValid,
            solutionAgeSec,
            shooterRollerEnabled,
            shooterAtTarget,
            hoodEnabled,
            hoodAtTarget,
            turretAutoAimActive,
            turretAtGoal,
            turretGoalLegal,
            setpointsFiniteAndBounded,
            acceptedVisionAgeSec,
            blockers
        );
    }

    public boolean ready() {
        return blockers.isEmpty();
    }

    /** Adds measured-motion reasons without conflating trajectory feasibility and feed readiness. */
    public ShotReadinessSnapshot withMotionBlockers(List<String> motionBlockers) {
        List<String> combined = new ArrayList<>(blockers);
        combined.addAll(motionBlockers);
        return new ShotReadinessSnapshot(timestampSec, enabledOperation, solutionValid, solutionAgeSec,
            shooterRollerEnabled, shooterAtTarget, hoodEnabled, hoodAtTarget, turretAutoAimActive,
            turretAtGoal, turretGoalLegal, setpointsFiniteAndBounded, acceptedVisionAgeSec, combined);
    }

    /** Override bypasses readiness, but never enabled-state, numeric, or actuator protections. */
    public boolean overrideSafe() {
        return enabledOperation && setpointsFiniteAndBounded && turretGoalLegal;
    }

    public boolean visionDegraded(double warningAgeSec) {
        return !Double.isFinite(acceptedVisionAgeSec) || acceptedVisionAgeSec > warningAgeSec;
    }

    public String blockerSummary() {
        return blockers.isEmpty() ? "ready" : String.join(",", blockers);
    }
}

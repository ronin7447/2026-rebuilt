package frc.robot.commands.compositions;

import frc.robot.util.sotm.ShotReadinessSnapshot;

/** Pure staged-feed state machine. All time values use one caller-provided monotonic clock. */
public final class FeedSequencer {
    private final double recoveryReadySec;
    private final boolean continueAfterReady;
    private boolean automaticFeedStarted;
    private final double outtakeDelaySec;
    private final double indexerDelaySec;
    private boolean recoveryRequired;
    private double stableReadySinceSec;
    private double stagingStartSec;
    private double lastTransitionSec;
    private State state;

    public FeedSequencer(double recoveryReadySec, double outtakeDelaySec, double indexerDelaySec) {
        this(recoveryReadySec, outtakeDelaySec, indexerDelaySec, false);
    }

    public FeedSequencer(double recoveryReadySec, double outtakeDelaySec, double indexerDelaySec,
        boolean continueAfterReady) {
        this.continueAfterReady = continueAfterReady;
        this.recoveryReadySec = recoveryReadySec;
        this.outtakeDelaySec = outtakeDelaySec;
        this.indexerDelaySec = indexerDelaySec;
        reset();
    }

    public void reset() {
        automaticFeedStarted = false;
        recoveryRequired = false;
        stableReadySinceSec = Double.NaN;
        stagingStartSec = Double.NaN;
        lastTransitionSec = Double.NaN;
        state = new State(false, false, false, false, "waiting-for-readiness", lastTransitionSec);
    }

    public State update(
        double nowSec,
        ShotReadinessSnapshot readiness,
        boolean overrideRequested,
        boolean interlockEnabled
    ) {
        boolean overrideActive = overrideRequested
            && (interlockEnabled ? readiness.overrideSafe() : readiness.enabledOperation());
        if (overrideActive) {
            // Force feed skips readiness, stability dwell and all startup/recovery staging.
            // Releasing override returns to automatic readiness and recovery staging.
            recoveryRequired = true;
            stableReadySinceSec = Double.NaN;
            stagingStartSec = Double.NaN;
            setState(nowSec, true, true, true, true, "manual-override");
            return state;
        }
        // Once automatic staging completes, ordinary ShotReady fluctuations are advisory.
        // Keep hard actuator/operation guards active and require readiness again after a hard stop.
        boolean continuationSafe = readiness.overrideSafe() && readiness.hoodEnabled()
            && readiness.turretAutoAimActive() && readiness.turretGoalLegal();
        if (interlockEnabled && continueAfterReady && automaticFeedStarted && continuationSafe) {
            setState(nowSec, true, true, true, false,
                readiness.ready() ? "ready" : "feeding-through-readiness-recovery");
            return state;
        }
        // A disabled interlock must not depend on any shooter/hood/turret readiness.
        // Each motor subsystem still enforces its own configuration and actuator limits.
        boolean permitted = interlockEnabled ? readiness.ready() : readiness.enabledOperation();
        if (interlockEnabled && continueAfterReady && automaticFeedStarted && !continuationSafe) permitted = false;
        if (!permitted) {
            automaticFeedStarted = false;
            if (Double.isFinite(stagingStartSec) || Double.isFinite(stableReadySinceSec)) recoveryRequired = true;
            stableReadySinceSec = Double.NaN;
            stagingStartSec = Double.NaN;
            setState(nowSec, false, false, false, overrideActive, readiness.blockerSummary());
            return state;
        }

        if (!Double.isFinite(stableReadySinceSec)) stableReadySinceSec = nowSec;
        double requiredStableSec = interlockEnabled && recoveryRequired ? recoveryReadySec : 0.0;
        if (nowSec - stableReadySinceSec + 1e-9 < requiredStableSec) {
            setState(nowSec, true, false, false, overrideActive, "recovering-readiness");
            return state;
        }
        if (!Double.isFinite(stagingStartSec)) stagingStartSec = nowSec;
        double elapsed = nowSec - stagingStartSec;
        boolean outtake = elapsed + 1e-9 >= outtakeDelaySec;
        boolean indexer = elapsed + 1e-9 >= outtakeDelaySec + indexerDelaySec;
        if (interlockEnabled && indexer) automaticFeedStarted = true;
        setState(nowSec, true, outtake, indexer, false,
            interlockEnabled ? "ready" : "readiness-bypassed");
        return state;
    }

    private void setState(
        double nowSec,
        boolean permitted,
        boolean outtake,
        boolean indexer,
        boolean overrideActive,
        String reason
    ) {
        if (state.permitted() != permitted
            || state.outtakeEnabled() != outtake
            || state.indexerEnabled() != indexer
            || state.overrideActive() != overrideActive
            || !state.reason().equals(reason)) {
            lastTransitionSec = nowSec;
        }
        state = new State(permitted, outtake, indexer, overrideActive, reason, lastTransitionSec);
    }

    public State state() {
        return state;
    }

    public record State(
        boolean permitted,
        boolean outtakeEnabled,
        boolean indexerEnabled,
        boolean overrideActive,
        String reason,
        double transitionTimestampSec
    ) {}
}

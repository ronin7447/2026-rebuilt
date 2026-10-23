package frc.robot.commands.compositions;

import frc.robot.util.sotm.ShotReadinessSnapshot;

/** Pure staged-feed state machine. All time values use one caller-provided monotonic clock. */
public final class FeedSequencer {
    private final double recoveryReadySec;
    private final double outtakeDelaySec;
    private final double indexerDelaySec;
    private boolean recoveryRequired;
    private double stableReadySinceSec;
    private double stagingStartSec;
    private double lastTransitionSec;
    private State state;

    public FeedSequencer(double recoveryReadySec, double outtakeDelaySec, double indexerDelaySec) {
        this.recoveryReadySec = recoveryReadySec;
        this.outtakeDelaySec = outtakeDelaySec;
        this.indexerDelaySec = indexerDelaySec;
        reset();
    }

    public void reset() {
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
        // A disabled interlock must not depend on any shooter/hood/turret readiness.
        // Each motor subsystem still enforces its own configuration and actuator limits.
        boolean permitted = interlockEnabled ? readiness.ready() : readiness.enabledOperation();
        if (!permitted) {
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

package frc.robot.util;

import java.util.ArrayDeque;
import java.util.Deque;

/** Pure stationary-step and rolling moving-target metrics in turret degrees. */
public final class TurretDiagnostics {
    private final double toleranceDeg;
    private final double settleTimeSec;
    private final double errorDeadbandDeg;
    private final double trackingWindowSec;
    private final Deque<TrackingSample> tracking = new ArrayDeque<>();
    private boolean stepActive;
    private double stepStartSec;
    private double startDeg;
    private double targetDeg;
    private double withinSinceSec = Double.NaN;
    private double settledSec = Double.NaN;
    private double maxOvershootDeg;
    private int signChanges;
    private int lastErrorSign;
    private double finalBiasDeg;

    public TurretDiagnostics(double toleranceDeg, double settleTimeSec, double errorDeadbandDeg, double trackingWindowSec) {
        this.toleranceDeg = toleranceDeg;
        this.settleTimeSec = settleTimeSec;
        this.errorDeadbandDeg = errorDeadbandDeg;
        this.trackingWindowSec = trackingWindowSec;
    }

    public void beginStep(double nowSec, double startDeg, double targetDeg) {
        stepActive = true;
        stepStartSec = nowSec;
        this.startDeg = startDeg;
        this.targetDeg = targetDeg;
        withinSinceSec = Double.NaN;
        settledSec = Double.NaN;
        maxOvershootDeg = 0.0;
        signChanges = 0;
        lastErrorSign = 0;
        finalBiasDeg = targetDeg - startDeg;
    }

    public void endStep() {
        stepActive = false;
        withinSinceSec = Double.NaN;
    }

    public void resetAll() {
        endStep();
        tracking.clear();
    }

    public Snapshot update(double nowSec, double finalGoalDeg, double profilePositionDeg, double measuredPositionDeg) {
        double goalErrorDeg = finalGoalDeg - measuredPositionDeg;
        double profileTrackingErrorDeg = profilePositionDeg - measuredPositionDeg;
        tracking.addLast(new TrackingSample(nowSec, goalErrorDeg, profileTrackingErrorDeg));
        while (!tracking.isEmpty() && nowSec - tracking.peekFirst().timestampSec() > trackingWindowSec) tracking.removeFirst();

        if (stepActive) {
            finalBiasDeg = targetDeg - measuredPositionDeg;
            double direction = Math.signum(targetDeg - startDeg);
            maxOvershootDeg = Math.max(maxOvershootDeg, Math.max(0.0, direction * (measuredPositionDeg - targetDeg)));
            int sign = Math.abs(finalBiasDeg) > errorDeadbandDeg ? (int) Math.signum(finalBiasDeg) : 0;
            if (sign != 0 && lastErrorSign != 0 && sign != lastErrorSign) signChanges++;
            if (sign != 0) lastErrorSign = sign;
            if (Math.abs(finalBiasDeg) <= toleranceDeg) {
                if (!Double.isFinite(withinSinceSec)) withinSinceSec = nowSec;
                if (!Double.isFinite(settledSec) && nowSec - withinSinceSec + 1e-9 >= settleTimeSec) settledSec = nowSec;
            } else {
                withinSinceSec = Double.NaN;
            }
        }

        double goalSquareSum = 0.0;
        double peakGoal = 0.0;
        double profileSquareSum = 0.0;
        double peakProfile = 0.0;
        for (TrackingSample sample : tracking) {
            goalSquareSum += sample.goalErrorDeg() * sample.goalErrorDeg();
            peakGoal = Math.max(peakGoal, Math.abs(sample.goalErrorDeg()));
            profileSquareSum += sample.profileErrorDeg() * sample.profileErrorDeg();
            peakProfile = Math.max(peakProfile, Math.abs(sample.profileErrorDeg()));
        }
        int count = tracking.size();
        return new Snapshot(
            stepActive,
            goalErrorDeg,
            profileTrackingErrorDeg,
            Double.isFinite(settledSec) ? settledSec - stepStartSec : Double.NaN,
            maxOvershootDeg,
            finalBiasDeg,
            signChanges,
            count == 0 ? Double.NaN : Math.sqrt(goalSquareSum / count),
            peakGoal,
            count == 0 ? Double.NaN : Math.sqrt(profileSquareSum / count),
            peakProfile,
            count
        );
    }

    private record TrackingSample(double timestampSec, double goalErrorDeg, double profileErrorDeg) {}

    public record Snapshot(
        boolean stationaryStepActive,
        double goalErrorDeg,
        double profileTrackingErrorDeg,
        double settlingTimeSec,
        double overshootDeg,
        double finalBiasDeg,
        int errorSignChanges,
        double rollingGoalErrorRmsDeg,
        double rollingGoalErrorPeakDeg,
        double rollingProfileErrorRmsDeg,
        double rollingProfileErrorPeakDeg,
        int rollingSampleCount
    ) {}
}

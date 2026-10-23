package frc.robot.util.sotm;

import java.util.Optional;

/** Brief fallback to a freshly revalidated trajectory; cached validity is never reused. */
public final class StableShotSolutionSelector {
    private final double maxAgeSec;
    private RobustShotSolution previous;
    private RobustShotRequest previousRequest;
    private double freshTimestamp = Double.NaN;

    public StableShotSolutionSelector(double maxAgeSec) {
        this.maxAgeSec = Math.max(0.0, maxAgeSec);
    }

    public Optional<RobustShotSolution> select(RobustShotRequest request,
        Optional<RobustShotSolution> fresh, double nowSec, boolean enabled) {
        if (!enabled || request == null || request.targetFieldPoint() == null
            || request.targetConstraints() == null || request.turretZeroOffset() == null
            || !Double.isFinite(nowSec)) {
            reset();
            return fresh;
        }
        if (previousRequest != null && (!previousRequest.targetFieldPoint().equals(request.targetFieldPoint())
            || previousRequest.targetConstraints().targetType() != request.targetConstraints().targetType()
            || previousRequest.targetConstraints().selectionMode() != request.targetConstraints().selectionMode()
            || !previousRequest.turretZeroOffset().equals(request.turretZeroOffset())
            || nowSec < freshTimestamp)) reset();
        if (fresh.isPresent() && fresh.get().quality().insideTolerance()) {
            previous = fresh.get();
            previousRequest = request;
            freshTimestamp = nowSec;
            return fresh;
        }
        if (previous != null && nowSec - freshTimestamp <= maxAgeSec + 1e-9) {
            Optional<RobustShotSolution> revalidated = RobustShotModel.revalidate(request, previous);
            if (revalidated.isPresent() && revalidated.get().quality().insideTolerance()) return revalidated;
        }
        return fresh;
    }

    public void reset() {
        previous = null;
        previousRequest = null;
        freshTimestamp = Double.NaN;
    }
}

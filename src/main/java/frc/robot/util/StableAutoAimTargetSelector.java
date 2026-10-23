package frc.robot.util;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.AutoAimTargeting.TargetSelection;

/** Confirms target changes while leaving continuous aiming within a target responsive. */
public final class StableAutoAimTargetSelector {
    private final double confirmationSec;
    private final double passbackAdvantageMeters;
    private final double maxSampleGapSec;
    private DriverStation.Alliance lastAlliance;
    private TargetSelection selected;
    private TargetSelection pending;
    private double pendingSince = Double.NaN;
    private double lastTime = Double.NaN;

    public StableAutoAimTargetSelector(double confirmationSec, double passbackAdvantageMeters,
        double maxSampleGapSec) {
        this.confirmationSec = Math.max(0.0, confirmationSec);
        this.passbackAdvantageMeters = Math.max(0.0, passbackAdvantageMeters);
        this.maxSampleGapSec = maxSampleGapSec;
    }

    public TargetSelection select(DriverStation.Alliance alliance, Translation2d robotPosition,
        double nowSec, boolean enabled) {
        TargetSelection raw = AutoAimTargeting.selectTarget(alliance, robotPosition, robotPosition.getY());
        if (!enabled || !Double.isFinite(nowSec)) {
            reset();
            return raw;
        }
        if (alliance != lastAlliance || (Double.isFinite(lastTime)
            && (nowSec <= lastTime || nowSec - lastTime > maxSampleGapSec))) reset();
        lastAlliance = alliance;
        lastTime = nowSec;
        if (selected == null) selected = raw;
        if (raw.equals(selected)) {
            clearPending();
            return selected;
        }
        if (raw.targetMode() == selected.targetMode()) {
            double oldDistance = Math.abs(robotPosition.getY() - selected.targetPoint().getY());
            double newDistance = Math.abs(robotPosition.getY() - raw.targetPoint().getY());
            if (oldDistance - newDistance <= passbackAdvantageMeters) {
                clearPending();
                return selected;
            }
        }
        if (!raw.equals(pending)) {
            pending = raw;
            pendingSince = nowSec;
        }
        if (nowSec - pendingSince + 1e-9 >= confirmationSec) {
            selected = raw;
            clearPending();
        }
        return selected;
    }

    private void clearPending() {
        pending = null;
        pendingSince = Double.NaN;
    }

    public void reset() {
        lastAlliance = null;
        selected = null;
        lastTime = Double.NaN;
        clearPending();
    }
}

package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Position hysteresis on a continuous legal angle; adds no time filter to chassis counterrotation. */
public final class TurretGoalStabilizer {
    private final double deadbandRotations;
    private final double maxSampleGapSec;
    private final double legalLow;
    private final double legalHigh;
    private double previousGoal = Double.NaN;
    private double previousTime = Double.NaN;

    public TurretGoalStabilizer(double deadbandDeg, double maxSampleGapSec,
        double legalLow, double legalHigh) {
        this.deadbandRotations = Math.max(0.0, deadbandDeg) / 360.0;
        this.maxSampleGapSec = maxSampleGapSec;
        this.legalLow = legalLow;
        this.legalHigh = legalHigh;
    }

    public Rotation2d calculate(Rotation2d requested, double currentRotations, double nowSec,
        boolean enabled) {
        if (!enabled) {
            reset();
            return requested;
        }
        if (!Double.isFinite(nowSec) || !Double.isFinite(currentRotations)
            || requested == null || !Double.isFinite(requested.getRotations())) {
            reset();
            return requested;
        }
        if (Double.isFinite(previousTime)
            && (nowSec <= previousTime || nowSec - previousTime > maxSampleGapSec)) reset();
        double reference = Double.isFinite(previousGoal) ? previousGoal : currentRotations;
        double candidate = TurretMath.getNearestEquivalentLegalTurretRotation(
            requested.getRotations(), reference, legalLow, legalHigh);
        // Compare with the last issued goal, not the last noisy input: slow real motion accumulates.
        if (!Double.isFinite(previousGoal)
            || Math.abs(candidate - previousGoal) > deadbandRotations + 1e-9) previousGoal = candidate;
        previousTime = nowSec;
        return Rotation2d.fromRotations(previousGoal);
    }

    public void reset() {
        previousGoal = Double.NaN;
        previousTime = Double.NaN;
    }
}

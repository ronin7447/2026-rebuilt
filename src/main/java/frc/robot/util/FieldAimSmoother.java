package frc.robot.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Time-based exponential smoothing along the shortest angular path. */
public final class FieldAimSmoother {
    private Rotation2d filtered;
    private double lastTime = Double.NaN;

    public Rotation2d calculate(Rotation2d input, double now, boolean enabled, double tau, double maxGap) {
        if (input == null || !Double.isFinite(input.getRadians()) || !Double.isFinite(now)) {
            reset();
            return input;
        }
        double dt = now - lastTime;
        if (!enabled || !Double.isFinite(tau) || tau <= 0 || filtered == null
            || !Double.isFinite(dt) || dt <= 0 || dt > maxGap) {
            filtered = input;
        } else {
            double alpha = -Math.expm1(-dt / tau);
            filtered = filtered.plus(Rotation2d.fromRadians(input.minus(filtered).getRadians() * alpha));
        }
        lastTime = now;
        return filtered;
    }

    public void reset() {
        filtered = null;
        lastTime = Double.NaN;
    }
}

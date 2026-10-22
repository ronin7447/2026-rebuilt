package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Limits the translation vector, including direction changes, while shooting is requested. */
public final class ShootingDriveLimiter {
    private ChassisSpeeds previous = new ChassisSpeeds();
    private double lastTimestampSec = Double.NaN;

    public void reset(ChassisSpeeds speeds, double nowSec) {
        previous = new ChassisSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond);
        lastTimestampSec = nowSec;
    }

    public ChassisSpeeds calculate(ChassisSpeeds requested, boolean shooting, double nowSec,
        double maxSpeedMps, double maxOmegaRadPerSec, double maxAccelMps2, double maxAlphaRadPerSec2) {
        if (!shooting) {
            reset(requested, nowSec);
            return previous;
        }
        Translation2d desired = new Translation2d(requested.vxMetersPerSecond, requested.vyMetersPerSecond);
        if (desired.getNorm() > maxSpeedMps) desired = desired.times(maxSpeedMps / desired.getNorm());
        double desiredOmega = Math.max(-maxOmegaRadPerSec, Math.min(maxOmegaRadPerSec, requested.omegaRadiansPerSecond));
        // A missed loop cannot authorize an arbitrarily large step on resumption.
        double dt = Double.isFinite(lastTimestampSec) ? Math.max(0.0, Math.min(0.02, nowSec - lastTimestampSec)) : 0.0;
        Translation2d prior = new Translation2d(previous.vxMetersPerSecond, previous.vyMetersPerSecond);
        Translation2d delta = desired.minus(prior);
        if (delta.getNorm() > maxAccelMps2 * dt) delta = delta.times(maxAccelMps2 * dt / delta.getNorm());
        Translation2d result = prior.plus(delta);
        double omegaStep = Math.max(-maxAlphaRadPerSec2 * dt,
            Math.min(maxAlphaRadPerSec2 * dt, desiredOmega - previous.omegaRadiansPerSecond));
        reset(new ChassisSpeeds(result.getX(), result.getY(), previous.omegaRadiansPerSecond + omegaStep), nowSec);
        return previous;
    }
}

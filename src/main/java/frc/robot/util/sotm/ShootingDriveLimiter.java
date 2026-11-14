package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Ramps translation speed while allowing immediate direction changes during shooting. */
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
        double priorSpeed = prior.getNorm();
        double desiredSpeed = desired.getNorm();
        double speedStep = Math.max(-maxAccelMps2 * dt,
            Math.min(maxAccelMps2 * dt, desiredSpeed - priorSpeed));
        double speed = priorSpeed + speedStep;
        // Steering follows the stick immediately; only speed magnitude is slew limited.
        // With a centered stick, retain the prior direction while ramping down to rest.
        Translation2d result = desiredSpeed > 0.0 ? desired.times(speed / desiredSpeed)
            : priorSpeed > 0.0 ? prior.times(speed / priorSpeed) : new Translation2d();
        double omegaStep = Math.max(-maxAlphaRadPerSec2 * dt,
            Math.min(maxAlphaRadPerSec2 * dt, desiredOmega - previous.omegaRadiansPerSecond));
        reset(new ChassisSpeeds(result.getX(), result.getY(), previous.omegaRadiansPerSecond + omegaStep), nowSec);
        return previous;
    }
}

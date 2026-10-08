package frc.robot.util.sotm;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Derivatives use odometry sample time, rather than command execution time. */
public final class ShotMotionEstimator {
    private final double alpha;
    private final double maxSampleDtSec;
    private final double maxLinearAccel;
    private final double maxAngularAccel;
    private Estimate estimate = new Estimate(Double.NaN, null, false, Double.NaN, Double.NaN);

    public ShotMotionEstimator(double alpha, double maxSampleDtSec,
        double maxLinearAccel, double maxAngularAccel) {
        this.alpha = alpha;
        this.maxSampleDtSec = maxSampleDtSec;
        this.maxLinearAccel = maxLinearAccel;
        this.maxAngularAccel = maxAngularAccel;
    }

    public Estimate sample(double timestampSec, Pose2d pose, ChassisSpeeds robotSpeeds,
        Rotation3d orientation) {
        if (!finite(timestampSec, pose, robotSpeeds, orientation)) {
            estimate = new Estimate(timestampSec, null, false, Double.NaN, Double.NaN);
            return estimate;
        }
        // A repeated read is not a new zero-acceleration observation.
        if (timestampSec == estimate.timestampSec()) return estimate;
        ChassisSpeeds speeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, pose.getRotation());
        double dt = timestampSec - estimate.timestampSec();
        boolean derivativeValid = estimate.motion() != null && dt > 1e-6 && dt <= maxSampleDtSec;
        double ax = 0.0;
        double ay = 0.0;
        double angular = 0.0;
        double rawLinear = Double.NaN;
        double rawAngular = Double.NaN;
        if (derivativeValid) {
            ShotMotionState previous = estimate.motion();
            double rawX = (speeds.vxMetersPerSecond - previous.fieldRelativeSpeeds().vxMetersPerSecond) / dt;
            double rawY = (speeds.vyMetersPerSecond - previous.fieldRelativeSpeeds().vyMetersPerSecond) / dt;
            double rawOmega = (speeds.omegaRadiansPerSecond - previous.fieldRelativeSpeeds().omegaRadiansPerSecond) / dt;
            rawLinear = Math.hypot(rawX, rawY);
            rawAngular = Math.abs(rawOmega);
            ax = filter(previous.fieldAccelXMps2(), rawX, maxLinearAccel);
            ay = filter(previous.fieldAccelYMps2(), rawY, maxLinearAccel);
            angular = filter(previous.angularAccelRadPerSec2(), rawOmega, maxAngularAccel);
        }
        ShotMotionState motion = new ShotMotionState(pose,
            new ChassisSpeeds(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond,
                robotSpeeds.omegaRadiansPerSecond),
            speeds, ax, ay, angular, orientation);
        estimate = new Estimate(timestampSec, motion, derivativeValid, rawLinear, rawAngular);
        return estimate;
    }

    public Estimate estimate() { return estimate; }

    private double filter(double previous, double raw, double maxMagnitude) {
        return previous + alpha * (Math.max(-maxMagnitude, Math.min(maxMagnitude, raw)) - previous);
    }

    private static boolean finite(double timestamp, Pose2d pose, ChassisSpeeds speeds, Rotation3d orientation) {
        return Double.isFinite(timestamp) && pose != null && speeds != null && orientation != null
            && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
            && Double.isFinite(pose.getRotation().getRadians())
            && Double.isFinite(speeds.vxMetersPerSecond) && Double.isFinite(speeds.vyMetersPerSecond)
            && Double.isFinite(speeds.omegaRadiansPerSecond)
            && Double.isFinite(orientation.getX()) && Double.isFinite(orientation.getY())
            && Double.isFinite(orientation.getZ());
    }

    public record Estimate(double timestampSec, ShotMotionState motion, boolean derivativeValid,
        double rawLinearAccelMps2, double rawAngularAccelRadPerSec2) {
        public List<String> shootingBlockers(double nowSec, double maxAgeSec, double latencySec,
            double maxSpeedMps, double maxAccelMps2, double maxOmegaRadPerSec, double maxAlphaRadPerSec2) {
            List<String> blockers = new ArrayList<>();
            double age = nowSec - timestampSec;
            if (motion == null || !derivativeValid || !Double.isFinite(age) || age < 0.0 || age > maxAgeSec) {
                return List.of("invalid-shot-motion");
            }
            ChassisSpeeds current = motion.fieldRelativeSpeeds();
            ChassisSpeeds release = ReleaseStatePredictor.predict(motion, latencySec + age).fieldRelativeSpeeds();
            if (Math.max(Math.hypot(current.vxMetersPerSecond, current.vyMetersPerSecond),
                Math.hypot(release.vxMetersPerSecond, release.vyMetersPerSecond)) > maxSpeedMps) {
                blockers.add("robot-speed");
            }
            // Never let low-pass filtering or predictor clamping hide a sudden maneuver.
            if (Math.max(rawLinearAccelMps2, Math.hypot(motion.fieldAccelXMps2(), motion.fieldAccelYMps2()))
                > maxAccelMps2) blockers.add("robot-acceleration");
            if (Math.max(Math.abs(current.omegaRadiansPerSecond), Math.abs(release.omegaRadiansPerSecond))
                > maxOmegaRadPerSec) blockers.add("robot-angular-speed");
            if (Math.max(rawAngularAccelRadPerSec2, Math.abs(motion.angularAccelRadPerSec2()))
                > maxAlphaRadPerSec2) blockers.add("robot-angular-acceleration");
            return List.copyOf(blockers);
        }
    }
}

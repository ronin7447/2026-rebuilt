package frc.robot.util;

public final class VisionTimestampPolicy {
    private VisionTimestampPolicy() {}

    public static boolean isValid(double nowSec, double captureSec, double maxFutureSec, double maxAgeSec) {
        if (!Double.isFinite(nowSec) || !Double.isFinite(captureSec)) return false;
        double ageSec = nowSec - captureSec;
        return ageSec >= -maxFutureSec && ageSec <= maxAgeSec;
    }

    public static double monotonicAcceptedTimestamp(double previousSec, double captureSec) {
        return Math.max(previousSec, captureSec);
    }
}

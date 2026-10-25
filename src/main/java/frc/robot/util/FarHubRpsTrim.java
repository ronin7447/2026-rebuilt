package frc.robot.util;

/** Empirical speed correction, smoothly introduced by release-to-Hub distance. */
public final class FarHubRpsTrim {
    private FarHubRpsTrim() {}

    public static double scale(double distance, boolean enabled, double start, double full, double multiplier) {
        if (!enabled || !Double.isFinite(distance) || !Double.isFinite(start) || !Double.isFinite(full)
            || !Double.isFinite(multiplier) || full <= start || multiplier <= 0) return 1.0;
        double blend = Math.max(0, Math.min(1, (distance - start) / (full - start)));
        blend = blend * blend * (3 - 2 * blend);
        return 1 + blend * (multiplier - 1);
    }
}

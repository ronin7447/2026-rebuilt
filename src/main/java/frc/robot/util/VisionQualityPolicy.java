package frc.robot.util;

/** Measurement-quality checks based on tags actually used by the pose solver. */
public final class VisionQualityPolicy {
    private VisionQualityPolicy() {}

    public static String rejectionReason(int usedTags, double distance, double ambiguity,
        double maxSingleDistance, double maxMultiDistance, double maxAmbiguity) {
        if (usedTags <= 0 || !Double.isFinite(distance) || distance <= 0) return "invalid-tag-distance";
        if (distance > (usedTags >= 2 ? maxMultiDistance : maxSingleDistance)) return "tag-range";
        if (usedTags == 1 && (!Double.isFinite(ambiguity) || ambiguity < 0 || ambiguity > maxAmbiguity))
            return "single-tag-ambiguity";
        return "";
    }

    public static double headingStdDev(boolean useVisionHeading, double scaledVisionStdDev) {
        // A large finite covariance gives vision negligible yaw authority without replacing
        // a capture-time yaw observation with an unrelated current-time gyro value.
        return useVisionHeading ? scaledVisionStdDev : 1.0e6;
    }
}

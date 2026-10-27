package frc.robot.util.sotm;

public record ShotQuality(
    boolean insideTolerance,
    String reason,
    double distanceMeters,
    double radialVelocityMps,
    double tangentialVelocityMps,
    double robustnessMargin,
    double polynomialFitError,
    double releaseLatencySec,
    double translationScale,
    double rotationScale
) {
    public static ShotQuality invalid(
        String reason,
        double distanceMeters,
        double radialVelocityMps,
        double tangentialVelocityMps,
        double releaseLatencySec
    ) {
        return new ShotQuality(
            false,
            reason,
            distanceMeters,
            radialVelocityMps,
            tangentialVelocityMps,
            Double.NaN,
            Double.NaN,
            releaseLatencySec,
            0.0,
            0.0
        );
    }
}

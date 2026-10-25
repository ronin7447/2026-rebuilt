package frc.robot.constants;

public final class GeneratedShotMapCoefficients {
    public static final double MIN_DISTANCE_METERS = 1.5;
    public static final double MAX_DISTANCE_METERS = 8;
    public static final double MIN_RADIAL_VELOCITY_MPS = -2.5;
    public static final double MAX_RADIAL_VELOCITY_MPS = 2.5;

    public static final double HOOD_FIT_MAX_ERROR_ROT = 0.553987;
    public static final double ROLLER_FIT_MAX_ERROR_RPS = 7.620648;
    public static final double MIN_ROBUSTNESS_MARGIN = 0.12;

    public static final double[] HOOD_POSITION_COEFFICIENTS = { -1.20204444582, -0.178394209212, 0.253852119698, 0.0110620791692, -0.0118928231451, -0.0162537295643 };
    public static final double[] ROLLER_RPS_COEFFICIENTS = { -49.5635839465, -7.36966628989, 5.44166110494, 0.224579363282, 0.118172366198, -0.218340841814 };

    private GeneratedShotMapCoefficients() {}

    public static double evaluateHoodPositionRot(double distanceMeters, double radialVelocityMps) {
        return evaluate(HOOD_POSITION_COEFFICIENTS, distanceMeters, radialVelocityMps);
    }

    public static double evaluateRollerRps(double distanceMeters, double radialVelocityMps) {
        return evaluate(ROLLER_RPS_COEFFICIENTS, distanceMeters, radialVelocityMps);
    }

    public static double estimateRobustnessMargin(double distanceMeters, double radialVelocityMps) {
        double normalizedDistance = Math.max(0.0, (distanceMeters - MIN_DISTANCE_METERS)
            / (MAX_DISTANCE_METERS - MIN_DISTANCE_METERS));
        double normalizedRadialVelocity = Math.abs(radialVelocityMps)
            / Math.max(Math.abs(MIN_RADIAL_VELOCITY_MPS), Math.abs(MAX_RADIAL_VELOCITY_MPS));
        return Math.max(0.0, 0.35 - (0.18 * normalizedDistance) - (0.08 * normalizedRadialVelocity));
    }

    public static boolean isInsideDomain(double distanceMeters, double radialVelocityMps) {
        return distanceMeters >= MIN_DISTANCE_METERS
            && distanceMeters <= MAX_DISTANCE_METERS
            && radialVelocityMps >= MIN_RADIAL_VELOCITY_MPS
            && radialVelocityMps <= MAX_RADIAL_VELOCITY_MPS;
    }

    private static double evaluate(double[] coefficients, double distanceMeters, double radialVelocityMps) {
        return coefficients[0]
            + (coefficients[1] * distanceMeters)
            + (coefficients[2] * radialVelocityMps)
            + (coefficients[3] * distanceMeters * distanceMeters)
            + (coefficients[4] * distanceMeters * radialVelocityMps)
            + (coefficients[5] * radialVelocityMps * radialVelocityMps);
    }
}

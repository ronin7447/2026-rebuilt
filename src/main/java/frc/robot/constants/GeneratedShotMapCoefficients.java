package frc.robot.constants;

public final class GeneratedShotMapCoefficients {
    public static final double MIN_DISTANCE_METERS = 1.5;
    public static final double MAX_DISTANCE_METERS = 8;
    public static final double MIN_RADIAL_VELOCITY_MPS = -2.5;
    public static final double MAX_RADIAL_VELOCITY_MPS = 2.5;

    public static final double HOOD_FIT_MAX_ERROR_ROT = 0.450818;
    public static final double ROLLER_FIT_MAX_ERROR_RPS = 7.775055;
    public static final double MIN_ROBUSTNESS_MARGIN = 0.12;

    public static final double[] HOOD_POSITION_COEFFICIENTS = { -1.07659815004, -0.209693374494, 0.257855638364, 0.0142339346282, -0.0123618779186, -0.0146389979100 };
    public static final double[] ROLLER_RPS_COEFFICIENTS = { -52.2213472219, -6.91206337270, 5.21374365988, 0.174222472420, 0.172620017257, -0.242745229749 };

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

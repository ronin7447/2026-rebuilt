package frc.robot.util.sotm;

/** Geometry and selection policy for one class of field target. */
public record ShotTargetConstraints(
    TargetType targetType,
    double targetHeightMeters,
    double goalRadiusMeters,
    double fuelRadiusMeters,
    double hubOpeningDepthMeters,
    double hubFrontLipHeightMeters,
    double hubLipClearanceMeters,
    double minEntryAngleDeg,
    boolean requiresHubOpeningClearance,
    double minDistanceMeters,
    double maxDistanceMeters,
    double minRadialVelocityMps,
    double maxRadialVelocityMps,
    SelectionMode selectionMode
) {
    public static ShotTargetConstraints hub(double targetHeightMeters) {
        return new ShotTargetConstraints(
            TargetType.HUB,
            targetHeightMeters,
            0.28,
            (5.91 * 0.0254) / 2.0,
            41.7 * 0.0254,
            72.0 * 0.0254,
            0.02,
            18.0,
            true,
            0.85,
            8.0,
            -2.5,
            2.5,
            SelectionMode.ROBUST
        );
    }

    public static ShotTargetConstraints passback(double targetHeightMeters) {
        return new ShotTargetConstraints(
            TargetType.PASSBACK,
            targetHeightMeters,
            0.24,
            (5.91 * 0.0254) / 2.0,
            0.0,
            0.0,
            0.0,
            -90.0,
            false,
            0.4,
            12.0,
            -2.5,
            2.5,
            SelectionMode.FLAT_FAST
        );
    }

    public enum TargetType { HUB, PASSBACK }

    public enum SelectionMode { ROBUST, FLAT_FAST }
}

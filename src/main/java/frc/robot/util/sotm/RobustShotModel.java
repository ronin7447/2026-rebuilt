package frc.robot.util.sotm;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.GeneratedShotMapCoefficients;
import frc.robot.util.AimMath;

public final class RobustShotModel {
    private static final double GRAVITY_MPS2 = 9.81;
    private static final double EPSILON = 1e-9;

    private RobustShotModel() {}

    public static Optional<RobustShotSolution> solve(RobustShotRequest request) {
        if (request == null
            || request.motionState() == null
            || request.robotToTurretPivot() == null
            || request.targetFieldPoint() == null
            || request.turretZeroOffset() == null
            || request.maxExitSpeedMps() <= 0.0
            || request.maxHoodPositionRot() < request.minHoodPositionRot()) {
            return Optional.empty();
        }

        PredictedReleaseState releaseState = ReleaseStatePredictor.predict(
            request.motionState(),
            request.releaseLatencySec()
        );
        Translation2d shooterFieldPosition = AimMath.computeTurretPivotFieldPosition(
            releaseState.robotPose(),
            request.robotToTurretPivot()
        );
        Translation2d target2d = new Translation2d(
            request.targetFieldPoint().getX(),
            request.targetFieldPoint().getY()
        );
        Translation2d delta2d = target2d.minus(shooterFieldPosition);
        double distanceMeters = delta2d.getNorm();
        if (distanceMeters <= EPSILON) {
            return Optional.empty();
        }

        ChassisSpeeds shooterRelativeSpeeds = computeShooterRelativeSpeeds(
            releaseState.robotRelativeSpeeds(),
            request.robotToTurretPivot()
        );
        ChassisSpeeds shooterFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            shooterRelativeSpeeds.vxMetersPerSecond,
            shooterRelativeSpeeds.vyMetersPerSecond,
            shooterRelativeSpeeds.omegaRadiansPerSecond,
            releaseState.robotPose().getRotation()
        );
        VelocityComponents velocityComponents = decomposeVelocity(delta2d, shooterFieldSpeeds);

        double radialVelocityMps = velocityComponents.radialVelocityMps();
        double tangentialVelocityMps = velocityComponents.tangentialVelocityMps();
        if (!GeneratedShotMapCoefficients.isInsideDomain(distanceMeters, radialVelocityMps)) {
            return Optional.of(invalidSolution(
                "outside-shot-map-domain",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        double baseHoodPositionRot = clamp(
            GeneratedShotMapCoefficients.evaluateHoodPositionRot(distanceMeters, radialVelocityMps),
            request.minHoodPositionRot(),
            request.maxHoodPositionRot()
        );
        double rollerRps = GeneratedShotMapCoefficients.evaluateRollerRps(distanceMeters, radialVelocityMps);
        double exitSpeedMps = rollerRpsToExitSpeedMps(rollerRps);
        if (exitSpeedMps <= 0.0 || exitSpeedMps > request.maxExitSpeedMps()) {
            return Optional.of(invalidSolution(
                "exit-speed-out-of-range",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        double fieldLaunchPitchDeg = launchAngleDegForHoodPosition(baseHoodPositionRot);
        double horizontalLaunchSpeed = exitSpeedMps * Math.cos(Math.toRadians(fieldLaunchPitchDeg));
        double verticalLaunchSpeed = exitSpeedMps * Math.sin(Math.toRadians(fieldLaunchPitchDeg));
        double requiredTangentialLaunchSpeed = -tangentialVelocityMps;
        if (Math.abs(requiredTangentialLaunchSpeed) >= horizontalLaunchSpeed) {
            return Optional.of(invalidSolution(
                "tangential-speed-too-high",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        double radialLaunchSpeed = Math.sqrt(
            (horizontalLaunchSpeed * horizontalLaunchSpeed)
                - (requiredTangentialLaunchSpeed * requiredTangentialLaunchSpeed)
        );
        double targetLineFieldSpeed = radialVelocityMps + radialLaunchSpeed;
        if (targetLineFieldSpeed <= EPSILON) {
            return Optional.of(invalidSolution(
                "field-speed-away-from-target",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        Translation2d unitToTarget = delta2d.div(distanceMeters);
        Translation2d tangentUnit = new Translation2d(-unitToTarget.getY(), unitToTarget.getX());
        Translation2d relativeHorizontalLaunchVelocity = unitToTarget.times(radialLaunchSpeed)
            .plus(tangentUnit.times(requiredTangentialLaunchSpeed));
        Translation3d launchVectorField = new Translation3d(
            relativeHorizontalLaunchVelocity.getX(),
            relativeHorizontalLaunchVelocity.getY(),
            verticalLaunchSpeed
        );
        Rotation2d localPitch = ShooterFrameTransform.localLaunchPitchForLaunchVector(
            launchVectorField,
            releaseState.robotOrientation()
        );
        double tiltCorrectedHoodPositionRot = hoodPositionForLaunchAngleDeg(localPitch.getDegrees());
        if (tiltCorrectedHoodPositionRot < request.minHoodPositionRot()
            || tiltCorrectedHoodPositionRot > request.maxHoodPositionRot()) {
            return Optional.of(invalidSolution(
                "tilt-corrected-hood-out-of-range",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        Rotation2d desiredFieldYaw = new Rotation2d(
            relativeHorizontalLaunchVelocity.getX(),
            relativeHorizontalLaunchVelocity.getY()
        );
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(shooterFieldPosition, target2d);
        Rotation2d desiredTurretYaw = ShooterFrameTransform.turretYawForLaunchVector(
            launchVectorField,
            releaseState.robotOrientation(),
            request.turretZeroOffset()
        );
        double flightTimeSec = estimateFlightTime(
            distanceMeters,
            targetLineFieldSpeed,
            request.targetFieldPoint().getZ() - request.shooterExitHeightMeters(),
            verticalLaunchSpeed
        );
        double robustnessMargin = GeneratedShotMapCoefficients.estimateRobustnessMargin(
            distanceMeters,
            radialVelocityMps
        );
        double fitError = Math.max(
            GeneratedShotMapCoefficients.HOOD_FIT_MAX_ERROR_ROT,
            GeneratedShotMapCoefficients.ROLLER_FIT_MAX_ERROR_RPS / 100.0
        );
        boolean insideTolerance = robustnessMargin >= GeneratedShotMapCoefficients.MIN_ROBUSTNESS_MARGIN;
        ShotQuality quality = new ShotQuality(
            insideTolerance,
            insideTolerance ? "ready" : "low-robustness-margin",
            distanceMeters,
            radialVelocityMps,
            tangentialVelocityMps,
            robustnessMargin,
            fitError,
            releaseState.releaseLatencySec(),
            insideTolerance ? 1.0 : 0.65,
            insideTolerance ? 1.0 : 0.55
        );

        return Optional.of(new RobustShotSolution(
            desiredFieldYaw,
            desiredTurretYaw,
            localPitch,
            tiltCorrectedHoodPositionRot,
            rollerRps,
            exitSpeedMps,
            flightTimeSec,
            desiredFieldYaw.minus(stationaryFieldYaw).getDegrees(),
            launchVectorField,
            quality
        ));
    }

    public static VelocityComponents decomposeVelocity(
        Translation2d shooterToTarget,
        ChassisSpeeds fieldRelativeSpeeds
    ) {
        double distanceMeters = shooterToTarget.getNorm();
        if (distanceMeters <= EPSILON) {
            return new VelocityComponents(0.0, 0.0);
        }
        Translation2d unitToTarget = shooterToTarget.div(distanceMeters);
        double radialVelocity = (fieldRelativeSpeeds.vxMetersPerSecond * unitToTarget.getX())
            + (fieldRelativeSpeeds.vyMetersPerSecond * unitToTarget.getY());
        double tangentialVelocity = (-unitToTarget.getY() * fieldRelativeSpeeds.vxMetersPerSecond)
            + (unitToTarget.getX() * fieldRelativeSpeeds.vyMetersPerSecond);
        return new VelocityComponents(radialVelocity, tangentialVelocity);
    }

    public static double launchAngleDegForHoodPosition(double hoodPositionRot) {
        return (hoodPositionRot - Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT)
            / Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_SLOPE;
    }

    public static double hoodPositionForLaunchAngleDeg(double launchAngleDeg) {
        return (Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_SLOPE * launchAngleDeg)
            + Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT;
    }

    public static double rollerRpsToExitSpeedMps(double rollerRps) {
        return (Math.abs(rollerRps) - Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT)
            / Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE;
    }

    private static RobustShotSolution invalidSolution(
        String reason,
        PredictedReleaseState releaseState,
        RobustShotRequest request,
        Translation2d shooterFieldPosition,
        Translation2d target2d,
        double distanceMeters,
        double radialVelocityMps,
        double tangentialVelocityMps
    ) {
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(shooterFieldPosition, target2d);
        Rotation2d stationaryTurretYaw = AimMath.computeTurretRelativeYaw(
            stationaryFieldYaw,
            releaseState.robotPose().getRotation(),
            request.turretZeroOffset()
        );
        ShotQuality quality = ShotQuality.invalid(
            reason,
            distanceMeters,
            radialVelocityMps,
            tangentialVelocityMps,
            releaseState.releaseLatencySec()
        );
        return new RobustShotSolution(
            stationaryFieldYaw,
            stationaryTurretYaw,
            Rotation2d.kZero,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            Double.NaN,
            0.0,
            new Translation3d(),
            quality
        );
    }

    private static ChassisSpeeds computeShooterRelativeSpeeds(
        ChassisSpeeds robotRelativeSpeeds,
        Translation2d robotToTurretPivot
    ) {
        double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
        double pivotVx = robotRelativeSpeeds.vxMetersPerSecond - (omega * robotToTurretPivot.getY());
        double pivotVy = robotRelativeSpeeds.vyMetersPerSecond + (omega * robotToTurretPivot.getX());
        return new ChassisSpeeds(pivotVx, pivotVy, omega);
    }

    private static double estimateFlightTime(
        double distanceMeters,
        double targetLineFieldSpeed,
        double targetHeightDeltaMeters,
        double verticalLaunchSpeed
    ) {
        double horizontalTimeSec = distanceMeters / targetLineFieldSpeed;
        double verticalTimeSec = solveVerticalFlightTime(targetHeightDeltaMeters, verticalLaunchSpeed);
        if (Double.isFinite(verticalTimeSec)) {
            return 0.5 * (horizontalTimeSec + verticalTimeSec);
        }
        return horizontalTimeSec;
    }

    private static double solveVerticalFlightTime(double heightDeltaMeters, double verticalLaunchSpeed) {
        double discriminant = (verticalLaunchSpeed * verticalLaunchSpeed)
            - (2.0 * GRAVITY_MPS2 * heightDeltaMeters);
        if (discriminant < 0.0) {
            return Double.NaN;
        }
        double root = Math.sqrt(discriminant);
        double ascendingTime = (verticalLaunchSpeed - root) / GRAVITY_MPS2;
        double descendingTime = (verticalLaunchSpeed + root) / GRAVITY_MPS2;
        if (descendingTime > EPSILON) {
            return descendingTime;
        }
        return ascendingTime > EPSILON ? ascendingTime : Double.NaN;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public record VelocityComponents(double radialVelocityMps, double tangentialVelocityMps) {}
}

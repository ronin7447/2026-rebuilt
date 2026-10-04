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
    private static final double DIRECT_MIN_EXIT_SPEED_MPS = 4.0;
    private static final double DIRECT_HOOD_STEP_ROT = 0.025;
    private static final double DIRECT_EXIT_SPEED_STEP_MPS = 0.05;
    private static final double EPSILON = 1e-9;

    private RobustShotModel() {}

    public static Optional<RobustShotSolution> solve(RobustShotRequest request) {
        long startNanos = System.nanoTime();
        if (request == null
            || request.motionState() == null
            || request.robotToTurretPivot() == null
            || request.targetFieldPoint() == null
            || request.turretZeroOffset() == null
            || request.targetConstraints() == null
            || request.motionState().robotPose() == null
            || request.motionState().robotRelativeSpeeds() == null
            || request.motionState().fieldRelativeSpeeds() == null
            || request.motionState().robotOrientation() == null
            || request.targetConstraints().targetType() == null
            || request.targetConstraints().selectionMode() == null
            || !isFiniteRequest(request)
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
        Optional<ShotCommand> shotCommand = shotCommandForInput(
            request, distanceMeters, radialVelocityMps, tangentialVelocityMps);
        if (shotCommand.isEmpty()) {
            String reason;
            if (request.targetConstraints().targetType() == ShotTargetConstraints.TargetType.PASSBACK) {
                reason = insideTargetDomain(request.targetConstraints(), distanceMeters, radialVelocityMps)
                    ? "no-feasible-passback-shot" : "outside-passback-domain";
            } else {
                reason = distanceMeters < GeneratedShotMapCoefficients.MIN_DISTANCE_METERS
                    ? "outside-shot-map-domain-close-range-no-direct-shot" : "outside-shot-map-domain";
            }
            return Optional.of(invalidSolution(
                reason,
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }

        double baseHoodPositionRot = shotCommand.get().hoodPositionRot();
        double rollerRps = shotCommand.get().rollerRps();
        if (Math.abs(rollerRps) > Constants.Shooter.MAX_ROLLER_RPS) {
            return Optional.of(invalidSolution(
                "roller-rps-out-of-range",
                releaseState,
                request,
                shooterFieldPosition,
                target2d,
                distanceMeters,
                radialVelocityMps,
                tangentialVelocityMps
            ));
        }
        double exitSpeedMps = shotCommand.get().exitSpeedMps();
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
        double flightTimeSec = distanceMeters / targetLineFieldSpeed;
        TrajectoryEvaluation trajectory = evaluateFinalTrajectory(
            request,
            distanceMeters,
            targetLineFieldSpeed,
            verticalLaunchSpeed
        );
        double robustnessMargin = GeneratedShotMapCoefficients.estimateRobustnessMargin(
            distanceMeters,
            radialVelocityMps
        );
        if (shotCommand.get().directRobustnessMargin() >= 0.0) {
            robustnessMargin = shotCommand.get().directRobustnessMargin();
        }
        double fitError = Math.max(
            GeneratedShotMapCoefficients.HOOD_FIT_MAX_ERROR_ROT,
            GeneratedShotMapCoefficients.ROLLER_FIT_MAX_ERROR_RPS / 100.0
        );
        boolean heuristicAccepted = request.targetConstraints().selectionMode() != ShotTargetConstraints.SelectionMode.ROBUST
            || robustnessMargin >= GeneratedShotMapCoefficients.MIN_ROBUSTNESS_MARGIN;
        boolean insideTolerance = trajectory.feasible() && heuristicAccepted;
        String resultReason = !trajectory.feasible()
            ? trajectory.reason()
            : (heuristicAccepted ? "ready" : "low-robustness-margin");
        ShotQuality quality = new ShotQuality(
            insideTolerance,
            resultReason,
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
            quality,
            request.targetConstraints().selectionMode(),
            shotCommand.get().source(),
            trajectory.feasible(),
            trajectory.reason(),
            flightTimeSec,
            (System.nanoTime() - startNanos) / 1.0e9
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

    private static Optional<ShotCommand> shotCommandForInput(
        RobustShotRequest request,
        double distanceMeters,
        double radialVelocityMps,
        double tangentialVelocityMps
    ) {
        ShotTargetConstraints constraints = request.targetConstraints();
        if (constraints.targetType() == ShotTargetConstraints.TargetType.HUB
            && GeneratedShotMapCoefficients.isInsideDomain(distanceMeters, radialVelocityMps)) {
            double hoodPositionRot = GeneratedShotMapCoefficients.evaluateHoodPositionRot(
                distanceMeters,
                radialVelocityMps
            );
            double rollerRps = GeneratedShotMapCoefficients.evaluateRollerRps(distanceMeters, radialVelocityMps);
            if (!Double.isFinite(hoodPositionRot)
                || hoodPositionRot < request.minHoodPositionRot()
                || hoodPositionRot > request.maxHoodPositionRot()
                || !Double.isFinite(rollerRps)) {
                return Optional.empty();
            }
            return Optional.of(new ShotCommand(
                hoodPositionRot,
                rollerRps,
                rollerRpsToExitSpeedMps(rollerRps),
                GeneratedShotMapCoefficients.estimateRobustnessMargin(distanceMeters, radialVelocityMps),
                CommandSource.GENERATED_POLYNOMIAL
            ));
        }

        if (insideTargetDomain(constraints, distanceMeters, radialVelocityMps)
            && constraints.targetType() == ShotTargetConstraints.TargetType.PASSBACK) {
            // Select against the actual compensated flight, not a stationary/radial-only shot
            // that loses downrange speed when some launch velocity is spent cancelling strafe.
            return solveDirectShot(request, distanceMeters, radialVelocityMps, tangentialVelocityMps);
        }
        if (insideTargetDomain(constraints, distanceMeters, radialVelocityMps)
            && distanceMeters < GeneratedShotMapCoefficients.MIN_DISTANCE_METERS) {
            return solveDirectShot(
                request,
                roundToStep(distanceMeters, 0.05),
                roundToStep(radialVelocityMps, 0.05),
                0.0
            );
        }

        return Optional.empty();
    }

    private static Optional<ShotCommand> solveDirectShot(
        RobustShotRequest request,
        double distanceMeters,
        double radialVelocityMps,
        double tangentialVelocityMps
    ) {
        ShotCommand best = null;
        for (double hoodPositionRot = request.minHoodPositionRot();
            hoodPositionRot <= request.maxHoodPositionRot() + EPSILON;
            hoodPositionRot += DIRECT_HOOD_STEP_ROT) {
            for (double exitSpeedMps = DIRECT_MIN_EXIT_SPEED_MPS;
                exitSpeedMps <= request.maxExitSpeedMps() + EPSILON;
                exitSpeedMps += DIRECT_EXIT_SPEED_STEP_MPS) {
                ShotCommand candidate = simulateDirectCandidate(
                    request,
                    distanceMeters,
                    radialVelocityMps,
                    tangentialVelocityMps,
                    hoodPositionRot,
                    exitSpeedMps
                );
                if (candidate != null && isBetterCandidate(request.targetConstraints().selectionMode(), candidate, best)) {
                    best = candidate;
                }
            }
        }
        return Optional.ofNullable(best);
    }

    private static ShotCommand simulateDirectCandidate(
        RobustShotRequest request,
        double distanceMeters,
        double radialVelocityMps,
        double tangentialVelocityMps,
        double hoodPositionRot,
        double exitSpeedMps
    ) {
        double launchAngleDeg = launchAngleDegForHoodPosition(hoodPositionRot);
        double launchAngleRad = Math.toRadians(launchAngleDeg);
        double horizontalLaunchSpeed = exitSpeedMps * Math.cos(launchAngleRad);
        double verticalLaunchSpeed = exitSpeedMps * Math.sin(launchAngleRad);
        if (Math.abs(tangentialVelocityMps) >= horizontalLaunchSpeed) return null;
        double targetLineFieldSpeed = Math.sqrt(horizontalLaunchSpeed * horizontalLaunchSpeed
            - tangentialVelocityMps * tangentialVelocityMps) + radialVelocityMps;
        if (targetLineFieldSpeed <= 0.05) {
            return null;
        }

        double flightTimeSec = distanceMeters / targetLineFieldSpeed;
        if (!Double.isFinite(flightTimeSec) || flightTimeSec <= 0.0 || flightTimeSec > 3.0) {
            return null;
        }

        TrajectoryEvaluation trajectory = evaluateFinalTrajectory(
            request,
            distanceMeters,
            targetLineFieldSpeed,
            verticalLaunchSpeed
        );
        if (!trajectory.feasible()) {
            return null;
        }

        double rollerRps = exitSpeedMpsToRollerRps(exitSpeedMps);
        if (Math.abs(rollerRps) > Constants.Shooter.MAX_ROLLER_RPS) {
            return null;
        }

        double impactHeightMeters = request.shooterExitHeightMeters()
            + (verticalLaunchSpeed * flightTimeSec)
            - (0.5 * GRAVITY_MPS2 * flightTimeSec * flightTimeSec);
        double heightErrorMeters = Math.abs(impactHeightMeters - request.targetConstraints().targetHeightMeters());
        double heightMarginMeters = request.targetConstraints().goalRadiusMeters() - heightErrorMeters;
        if (heightMarginMeters < 0.0) {
            return null;
        }

        double robustnessMargin = scoreDirectRobustness(
            request,
            distanceMeters,
            hoodPositionRot,
            exitSpeedMps,
            heightMarginMeters,
            flightTimeSec
        );
        return new ShotCommand(
            hoodPositionRot,
            rollerRps,
            exitSpeedMps,
            robustnessMargin,
            CommandSource.DIRECT_CANDIDATE
        );
    }

    private static TrajectoryEvaluation evaluateFinalTrajectory(
        RobustShotRequest request,
        double distanceMeters,
        double targetLineFieldSpeed,
        double verticalLaunchSpeed
    ) {
        ShotTargetConstraints constraints = request.targetConstraints();
        if (!Double.isFinite(targetLineFieldSpeed) || targetLineFieldSpeed <= 0.05) {
            return new TrajectoryEvaluation(false, "target-plane-crossing-unavailable");
        }
        double halfOpeningDepth = constraints.requiresHubOpeningClearance()
            ? constraints.hubOpeningDepthMeters() / 2.0
            : 0.18;
        double frontDistance = Math.max(0.0, distanceMeters - halfOpeningDepth);
        double backDistance = distanceMeters + halfOpeningDepth;
        boolean openingIntersectionValid = false;
        for (int index = 0; index < 9; index++) {
            double sampleDistance = frontDistance + ((backDistance - frontDistance) * index / 8.0);
            double sampleTime = sampleDistance / targetLineFieldSpeed;
            double sampleHeight = request.shooterExitHeightMeters()
                + (verticalLaunchSpeed * sampleTime)
                - (0.5 * GRAVITY_MPS2 * sampleTime * sampleTime);
            if (sampleHeight >= constraints.targetHeightMeters() - constraints.goalRadiusMeters()
                && sampleHeight <= constraints.targetHeightMeters() + constraints.goalRadiusMeters()) {
                openingIntersectionValid = true;
                break;
            }
        }
        if (!openingIntersectionValid) {
            return new TrajectoryEvaluation(false, "target-opening-miss");
        }
        if (!constraints.requiresHubOpeningClearance()) {
            return new TrajectoryEvaluation(true, "ready");
        }

        double lipDistanceMeters = Math.max(0.0, distanceMeters - halfOpeningDepth);
        double lipTimeSec = lipDistanceMeters / targetLineFieldSpeed;
        double lipHeightMeters = request.shooterExitHeightMeters()
            + (verticalLaunchSpeed * lipTimeSec)
            - (0.5 * GRAVITY_MPS2 * lipTimeSec * lipTimeSec);
        double minimumBallCenterHeight = constraints.hubFrontLipHeightMeters()
            + constraints.fuelRadiusMeters()
            + constraints.hubLipClearanceMeters();
        if (lipHeightMeters < minimumBallCenterHeight) {
            return new TrajectoryEvaluation(false, "hub-front-lip-clearance");
        }
        double targetTimeSec = distanceMeters / targetLineFieldSpeed;
        double verticalVelocityAtTarget = verticalLaunchSpeed - (GRAVITY_MPS2 * targetTimeSec);
        double entryAngleDeg = Math.toDegrees(Math.atan2(-verticalVelocityAtTarget, targetLineFieldSpeed));
        if (entryAngleDeg < constraints.minEntryAngleDeg()) {
            return new TrajectoryEvaluation(false, "hub-entry-angle");
        }
        return new TrajectoryEvaluation(true, "ready");
    }

    private static boolean isBetterCandidate(
        ShotTargetConstraints.SelectionMode selectionMode,
        ShotCommand candidate,
        ShotCommand best
    ) {
        if (best == null) return true;
        if (selectionMode == ShotTargetConstraints.SelectionMode.FLAT_FAST) {
            double candidateAngle = launchAngleDegForHoodPosition(candidate.hoodPositionRot());
            double bestAngle = launchAngleDegForHoodPosition(best.hoodPositionRot());
            double candidateHorizontal = candidate.exitSpeedMps() * Math.cos(Math.toRadians(candidateAngle));
            double bestHorizontal = best.exitSpeedMps() * Math.cos(Math.toRadians(bestAngle));
            if (candidateHorizontal > bestHorizontal + EPSILON) return true;
            if (Math.abs(candidateHorizontal - bestHorizontal) <= EPSILON
                && candidateAngle < bestAngle - EPSILON) return true;
            return Math.abs(candidateHorizontal - bestHorizontal) <= EPSILON
                && Math.abs(candidateAngle - bestAngle) <= EPSILON
                && candidate.exitSpeedMps() > best.exitSpeedMps() + EPSILON;
        }
        return candidate.directRobustnessMargin() > best.directRobustnessMargin();
    }

    private static double scoreDirectRobustness(
        RobustShotRequest request,
        double distanceMeters,
        double hoodPositionRot,
        double exitSpeedMps,
        double heightMarginMeters,
        double flightTimeSec
    ) {
        double hoodCenter = (request.minHoodPositionRot() + request.maxHoodPositionRot()) / 2.0;
        double hoodSpan = Math.max((request.maxHoodPositionRot() - request.minHoodPositionRot()) / 2.0, EPSILON);
        double hoodEdgePenalty = Math.abs(hoodPositionRot - hoodCenter) / hoodSpan;
        double speedHeadroom = (request.maxExitSpeedMps() - exitSpeedMps)
            / Math.max(request.maxExitSpeedMps() - DIRECT_MIN_EXIT_SPEED_MPS, EPSILON);
        double scoreMaxDistance = request.targetConstraints().targetType() == ShotTargetConstraints.TargetType.HUB
            ? GeneratedShotMapCoefficients.MIN_DISTANCE_METERS
            : request.targetConstraints().maxDistanceMeters();
        double distancePenalty = (distanceMeters - request.targetConstraints().minDistanceMeters())
            / Math.max(scoreMaxDistance - request.targetConstraints().minDistanceMeters(), EPSILON);
        return heightMarginMeters
            + (0.08 * speedHeadroom)
            - (0.08 * flightTimeSec)
            - (0.05 * hoodEdgePenalty)
            - (0.04 * distancePenalty);
    }

    private static double exitSpeedMpsToRollerRps(double exitSpeedMps) {
        return Constants.Shooter.ROLLER_DIRECTION_SIGN
            * ((Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE * exitSpeedMps)
                + Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT);
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
            quality,
            request.targetConstraints().selectionMode(),
            CommandSource.NONE,
            false,
            reason,
            Double.NaN,
            0.0
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

    private static boolean isFiniteRequest(RobustShotRequest request) {
        ShotMotionState motion = request.motionState();
        return Double.isFinite(request.shooterExitHeightMeters())
            && Double.isFinite(request.robotToTurretPivot().getX())
            && Double.isFinite(request.robotToTurretPivot().getY())
            && Double.isFinite(request.targetFieldPoint().getX())
            && Double.isFinite(request.targetFieldPoint().getY())
            && Double.isFinite(request.targetFieldPoint().getZ())
            && Double.isFinite(request.minHoodPositionRot())
            && Double.isFinite(request.maxHoodPositionRot())
            && Double.isFinite(request.maxExitSpeedMps())
            && Double.isFinite(request.releaseLatencySec())
            && Double.isFinite(request.turretZeroOffset().getRadians())
            && Double.isFinite(motion.robotPose().getX())
            && Double.isFinite(motion.robotPose().getY())
            && Double.isFinite(motion.robotPose().getRotation().getRadians())
            && Double.isFinite(motion.robotRelativeSpeeds().vxMetersPerSecond)
            && Double.isFinite(motion.robotRelativeSpeeds().vyMetersPerSecond)
            && Double.isFinite(motion.robotRelativeSpeeds().omegaRadiansPerSecond)
            && Double.isFinite(motion.fieldRelativeSpeeds().vxMetersPerSecond)
            && Double.isFinite(motion.fieldRelativeSpeeds().vyMetersPerSecond)
            && Double.isFinite(motion.fieldRelativeSpeeds().omegaRadiansPerSecond)
            && Double.isFinite(motion.fieldAccelXMps2())
            && Double.isFinite(motion.fieldAccelYMps2())
            && Double.isFinite(motion.angularAccelRadPerSec2())
            && Double.isFinite(motion.robotOrientation().getX())
            && Double.isFinite(motion.robotOrientation().getY())
            && Double.isFinite(motion.robotOrientation().getZ())
            && Double.isFinite(request.targetConstraints().targetHeightMeters())
            && Double.isFinite(request.targetConstraints().goalRadiusMeters())
            && Double.isFinite(request.targetConstraints().minDistanceMeters())
            && Double.isFinite(request.targetConstraints().maxDistanceMeters())
            && Double.isFinite(request.targetConstraints().minRadialVelocityMps())
            && Double.isFinite(request.targetConstraints().maxRadialVelocityMps())
            && Double.isFinite(request.targetConstraints().hubOpeningDepthMeters())
            && Double.isFinite(request.targetConstraints().hubFrontLipHeightMeters())
            && Double.isFinite(request.targetConstraints().fuelRadiusMeters())
            && Double.isFinite(request.targetConstraints().hubLipClearanceMeters())
            && Double.isFinite(request.targetConstraints().minEntryAngleDeg());
    }

    private static double roundToStep(double value, double step) {
        return Math.round(value / step) * step;
    }

    private static boolean insideTargetDomain(ShotTargetConstraints constraints,
        double distanceMeters, double radialVelocityMps) {
        return distanceMeters >= constraints.minDistanceMeters()
            && distanceMeters <= constraints.maxDistanceMeters()
            && radialVelocityMps >= constraints.minRadialVelocityMps()
            && radialVelocityMps <= constraints.maxRadialVelocityMps();
    }

    public record VelocityComponents(double radialVelocityMps, double tangentialVelocityMps) {}

    private record ShotCommand(
        double hoodPositionRot,
        double rollerRps,
        double exitSpeedMps,
        double directRobustnessMargin,
        CommandSource source
    ) {}

    private record TrajectoryEvaluation(boolean feasible, String reason) {}
}

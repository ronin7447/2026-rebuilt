package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;
import frc.robot.constants.GeneratedShotMapCoefficients;

class RobustShotModelTest {
    private static final double EPSILON = 1e-6;

    @Test
    void acceleratingRotatingShotReconstructsScoringTrajectoryFromActuatorCommands() {
        double latency = 0.14;
        double heading = 0.2;
        Pose2d pose = new Pose2d(0, 0, Rotation2d.fromRadians(heading));
        ChassisSpeeds field = new ChassisSpeeds(0.4, 0.3, 0.3);
        Translation2d pivot = new Translation2d(0.2, 0.1);
        ShotMotionState motion = new ShotMotionState(pose,
            ChassisSpeeds.fromFieldRelativeSpeeds(field, pose.getRotation()), field,
            0.5, -0.4, 0.2, new Rotation3d(0.01, 0.02, heading));
        var request = new RobustShotRequest(motion, pivot, Constants.Shooter.EXIT_HEIGHT_METERS,
            new Translation3d(4, 0, Constants.FieldTargets.HUB_TARGET_HEIGHT_METERS), Rotation2d.kZero,
            Constants.Shooter.MIN_HOOD_POSITION_ROT, Constants.Shooter.MAX_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_EXIT_SPEED_MPS, latency);
        var solution = RobustShotModel.solve(request).orElseThrow();
        assertTrue(solution.quality().insideTolerance(), solution.quality().reason());

        // Independently reconstruct departure and launch from turret, hood and RPS commands.
        double releaseHeading = heading + 0.3 * latency + 0.5 * 0.2 * latency * latency;
        Translation2d offset = pivot.rotateBy(Rotation2d.fromRadians(releaseHeading));
        Translation2d releasePosition = new Translation2d(0.4 * latency + 0.25 * latency * latency,
            0.3 * latency - 0.2 * latency * latency).plus(offset);
        double omega = 0.3 + 0.2 * latency;
        Translation3d inherited = new Translation3d(0.4 + 0.5 * latency - omega * offset.getY(),
            0.3 - 0.4 * latency + omega * offset.getX(), 0);
        double speed = (Math.abs(solution.rollerRps()) - Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT)
            / Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE;
        double pitch = Math.toRadians((solution.hoodPositionRot() - Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT)
            / Constants.Shooter.HOOD_POSITION_PER_LAUNCH_DEG_SLOPE);
        double yaw = solution.desiredTurretYaw().getRadians();
        Translation3d relative = new Translation3d(speed * Math.cos(pitch) * Math.cos(yaw),
            speed * Math.cos(pitch) * Math.sin(yaw), speed * Math.sin(pitch))
            .rotateBy(new Rotation3d(0.01, 0.02, releaseHeading));
        Translation3d velocity = relative.plus(inherited);
        double time = (4 - releasePosition.getX()) / velocity.getX();
        assertEquals(0, releasePosition.getY() + velocity.getY() * time, 1e-6);
        assertEquals(solution.flightTimeSec(), time, 1e-6);
        double z = Constants.Shooter.EXIT_HEIGHT_METERS + velocity.getZ() * time - 4.905 * time * time;
        assertTrue(Math.abs(z - request.targetFieldPoint().getZ()) <= request.targetConstraints().goalRadiusMeters());
        double entryAngle = Math.toDegrees(Math.atan2(9.81 * time - velocity.getZ(),
            Math.hypot(velocity.getX(), velocity.getY())));
        assertTrue(entryAngle >= request.targetConstraints().minEntryAngleDeg());
        double frontTime = Math.max(0, time - request.targetConstraints().hubOpeningDepthMeters()
            / (2 * Math.hypot(velocity.getX(), velocity.getY())));
        double frontHeight = Constants.Shooter.EXIT_HEIGHT_METERS + velocity.getZ() * frontTime
            - 4.905 * frontTime * frontTime;
        assertTrue(frontHeight >= request.targetConstraints().hubFrontLipHeightMeters()
            + request.targetConstraints().fuelRadiusMeters() + request.targetConstraints().hubLipClearanceMeters());

        ShotMotionState noAcceleration = new ShotMotionState(pose, motion.robotRelativeSpeeds(), field,
            0, 0, 0, motion.robotOrientation());
        var unaccelerated = RobustShotModel.solve(new RobustShotRequest(noAcceleration, pivot,
            request.shooterExitHeightMeters(), request.targetFieldPoint(), request.turretZeroOffset(),
            request.minHoodPositionRot(), request.maxHoodPositionRot(), request.maxExitSpeedMps(), latency)).orElseThrow();
        assertTrue(Math.abs(solution.desiredTurretYaw().minus(unaccelerated.desiredTurretYaw()).getRadians()) > 1e-4);
        assertTrue(Math.abs(solution.rollerRps() - unaccelerated.rollerRps()) > 1e-4);
    }

    @Test
    void decomposesRadialAndTangentialVelocity() {
        RobustShotModel.VelocityComponents components = RobustShotModel.decomposeVelocity(
            new Translation2d(4.0, 0.0),
            new ChassisSpeeds(1.25, -0.75, 0.0)
        );

        assertEquals(1.25, components.radialVelocityMps(), EPSILON);
        assertEquals(-0.75, components.tangentialVelocityMps(), EPSILON);
    }

    @Test
    void generatedPolynomialsEvaluateInsideDomain() {
        double hood = GeneratedShotMapCoefficients.evaluateHoodPositionRot(4.0, 0.0);
        double roller = GeneratedShotMapCoefficients.evaluateRollerRps(4.0, 0.0);

        assertTrue(GeneratedShotMapCoefficients.isInsideDomain(4.0, 0.0));
        assertTrue(hood >= Constants.Shooter.MIN_HOOD_POSITION_ROT);
        assertTrue(hood <= Constants.Shooter.MAX_HOOD_POSITION_ROT);
        assertTrue(roller < 0.0);
        assertTrue(Math.abs(roller) <= Constants.Shooter.MAX_ROLLER_RPS);
    }

    @Test
    void configuredMaxExitSpeedRespectsRollerCap() {
        double rollerMagnitude = (Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE
            * Constants.Shooter.MAX_EXIT_SPEED_MPS)
            + Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT;

        assertTrue(rollerMagnitude <= Constants.Shooter.MAX_ROLLER_RPS + EPSILON);
    }

    @Test
    void zeroTiltSolutionStaysNearTargetLine() {
        Optional<RobustShotSolution> solution = RobustShotModel.solve(requestFor(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        ));

        assertTrue(solution.isPresent());
        assertTrue(solution.get().quality().insideTolerance());
        assertEquals(0.0, solution.get().desiredFieldYaw().getDegrees(), 1.0);
        assertEquals(solution.get().desiredFieldYaw().getDegrees(), solution.get().desiredTurretYaw().getDegrees(), 1.0);
    }

    @Test
    void tiltCompensationChangesLocalLaunchPitch() {
        RobustShotSolution flat = RobustShotModel.solve(requestFor(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        )).orElseThrow();
        RobustShotSolution pitched = RobustShotModel.solve(requestFor(
            new Pose2d(0.0, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d(0.0, Math.toRadians(8.0), 0.0)
        )).orElseThrow();

        assertFalse(Double.isNaN(pitched.hoodPositionRot()));
        assertTrue(Math.abs(flat.recommendedLaunchPitch().getDegrees()
            - pitched.recommendedLaunchPitch().getDegrees()) > 1.0);
    }

    @Test
    void farShotUsesRobustPolynomialInsteadOfLowestSpeedBias() {
        RobustShotSolution close = RobustShotModel.solve(requestFor(
            new Pose2d(2.0, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        )).orElseThrow();
        RobustShotSolution far = RobustShotModel.solve(requestFor(
            new Pose2d(-2.0, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        )).orElseThrow();

        assertTrue(Math.abs(far.rollerRps()) > Math.abs(close.rollerRps()));
        assertTrue(far.flightTimeSec() < 3.0);
    }

    @Test
    void disabledTiltMatchesLevelShotAndRevalidationWhilePreservingYawAndMotion() {
        Pose2d pose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(35));
        RobustShotRequest tilted = requestFor(pose, new ChassisSpeeds(),
            new Rotation3d(Math.toRadians(20), Math.toRadians(30), pose.getRotation().getRadians()));
        ShotMotionState measured = tilted.motionState();
        ShotMotionState level = measured.withTiltCompensation(false);
        assertTrue(measured == measured.withTiltCompensation(true));
        assertEquals(measured.robotPose(), level.robotPose());
        assertEquals(measured.robotRelativeSpeeds(), level.robotRelativeSpeeds());
        assertEquals(measured.fieldRelativeSpeeds(), level.fieldRelativeSpeeds());
        assertEquals(measured.fieldAccelXMps2(), level.fieldAccelXMps2());
        assertEquals(measured.fieldAccelYMps2(), level.fieldAccelYMps2());
        assertEquals(measured.angularAccelRadPerSec2(), level.angularAccelRadPerSec2());
        assertEquals(0, level.robotOrientation().getX(), EPSILON);
        assertEquals(0, level.robotOrientation().getY(), EPSILON);
        assertEquals(measured.robotOrientation().getZ(), level.robotOrientation().getZ(), EPSILON);
        RobustShotRequest corrected = new RobustShotRequest(level, tilted.robotToTurretPivot(),
            tilted.shooterExitHeightMeters(), tilted.targetFieldPoint(), tilted.turretZeroOffset(),
            tilted.minHoodPositionRot(), tilted.maxHoodPositionRot(), tilted.maxExitSpeedMps(),
            tilted.releaseLatencySec(), tilted.targetConstraints());
        RobustShotSolution expected = RobustShotModel.solve(requestFor(pose, new ChassisSpeeds(),
            new Rotation3d(0, 0, pose.getRotation().getRadians()))).orElseThrow();
        RobustShotSolution fresh = RobustShotModel.solve(corrected).orElseThrow();
        RobustShotSolution reused = RobustShotModel.revalidate(corrected, expected).orElseThrow();
        assertTrue(fresh.quality().insideTolerance(), fresh.quality().reason());
        assertTrue(reused.quality().insideTolerance(), reused.quality().reason());
        assertEquals(expected.hoodPositionRot(), fresh.hoodPositionRot(), EPSILON);
        assertEquals(expected.desiredTurretYaw().getRadians(), fresh.desiredTurretYaw().getRadians(), EPSILON);
        assertEquals(expected.rollerRps(), fresh.rollerRps(), EPSILON);
        assertEquals(expected.hoodPositionRot(), reused.hoodPositionRot(), EPSILON);
        assertEquals(expected.desiredTurretYaw().getRadians(), reused.desiredTurretYaw().getRadians(), EPSILON);
    }

    @Test
    void closeRangeHubShotUsesDirectSweepAbovePolynomialDomain() {
        RobustShotSolution close = RobustShotModel.solve(requestFor(
            new Pose2d(3.1, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        )).orElseThrow();

        assertTrue(close.quality().insideTolerance());
        assertTrue(close.recommendedLaunchPitch().getDegrees() > 75.0);
        assertTrue(close.recommendedLaunchPitch().getDegrees() <= 80.0);
    }

    @Test
    void passbackUsesFlatFastDirectCandidateAndFinalCrossingTime() {
        Pose2d pose = new Pose2d(6.0, 4.0, Rotation2d.kZero);
        ChassisSpeeds fieldSpeeds = new ChassisSpeeds(0.5, -0.4, 0.0);
        ShotMotionState motion = new ShotMotionState(
            pose,
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation()),
            fieldSpeeds,
            0.0,
            0.0,
            0.0,
            new Rotation3d()
        );
        RobustShotSolution solution = RobustShotModel.solve(new RobustShotRequest(
            motion,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Constants.Shooter.EXIT_HEIGHT_METERS,
            new Translation3d(0.48, 6.0, Constants.FieldTargets.BACK_TARGET_HEIGHT_METERS),
            Rotation2d.kZero,
            Constants.Shooter.MIN_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_EXIT_SPEED_MPS,
            0.14,
            ShotTargetConstraints.passback(Constants.FieldTargets.BACK_TARGET_HEIGHT_METERS)
        )).orElseThrow();

        assertEquals(CommandSource.DIRECT_CANDIDATE, solution.commandSource());
        assertEquals(ShotTargetConstraints.SelectionMode.FLAT_FAST, solution.selectionMode());
        assertEquals(solution.flightTimeSec(), solution.targetPlaneCrossingTimeSec(), EPSILON);
        assertEquals(solution.trajectoryFeasible(), solution.quality().insideTolerance());
    }

    @Test
    void nonfiniteCriticalInputIsRejectedAtBoundary() {
        RobustShotRequest request = requestFor(
            new Pose2d(Double.NaN, 0.0, Rotation2d.kZero),
            new ChassisSpeeds(),
            new Rotation3d()
        );
        assertTrue(RobustShotModel.solve(request).isEmpty());
    }

    @Test
    void farFlatShotsReachOpeningWithRadialAndSidewaysMotion() {
        for (ChassisSpeeds speeds : new ChassisSpeeds[] {
            new ChassisSpeeds(), new ChassisSpeeds(1.0, 0.7, 0.0), new ChassisSpeeds(-1.0, -0.7, 0.0)
        }) {
            var request = requestFor(new Pose2d(-2.0, 0.0, Rotation2d.kZero), speeds, new Rotation3d());
            var shot = RobustShotModel.solve(request).orElseThrow();
            assertTrue(shot.quality().insideTolerance(), shot.quality().reason());
            assertEquals(Constants.ShotMap.FAR_HUB_SHOT_MAP_TRIAL == 2 ? 61.0 : 63.0,
                shot.recommendedLaunchPitch().getDegrees(), EPSILON);
            Translation3d velocity = shot.launchVectorField().plus(
                new Translation3d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, 0.0));
            double t = shot.flightTimeSec();
            assertEquals(6.0, velocity.getX() * t, EPSILON);
            assertEquals(0.0, velocity.getY() * t, EPSILON);
            assertEquals(request.targetFieldPoint().getZ(), request.shooterExitHeightMeters()
                + velocity.getZ() * t - 4.905 * t * t, request.targetConstraints().goalRadiusMeters());
            assertTrue(Math.abs(shot.rollerRps()) <= Constants.Shooter.MAX_ROLLER_RPS);
            assertTrue(RobustShotModel.revalidate(request, shot).orElseThrow().quality().insideTolerance());
        }
    }

    @Test
    void farFlatTransitionIsContinuousAndCloseShotsKeepMap() {
        RobustShotSolution prior = null;
        for (double distance : new double[] {4.0, 4.8999, 4.9, 4.9001, 5.4999, 5.5, 5.5001}) {
            var shot = RobustShotModel.solve(requestFor(
                new Pose2d(4.0 - distance, 0.0, Rotation2d.kZero), new ChassisSpeeds(), new Rotation3d())).orElseThrow();
            assertTrue(shot.quality().insideTolerance(), shot.quality().reason());
            if (distance <= 4.9) {
                assertEquals(GeneratedShotMapCoefficients.evaluateRollerRps(distance, 0.0), shot.rollerRps(), EPSILON);
                assertEquals(CommandSource.GENERATED_POLYNOMIAL, shot.commandSource());
            }
            if (prior != null && distance != 5.4999 && distance != 4.8999) {
                assertEquals(prior.hoodPositionRot(), shot.hoodPositionRot(), 0.001);
                assertEquals(prior.rollerRps(), shot.rollerRps(), 0.01);
            }
            prior = shot;
        }
    }

    private RobustShotRequest requestFor(
        Pose2d robotPose,
        ChassisSpeeds robotRelativeSpeeds,
        Rotation3d robotOrientation
    ) {
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            robotRelativeSpeeds.vxMetersPerSecond,
            robotRelativeSpeeds.vyMetersPerSecond,
            robotRelativeSpeeds.omegaRadiansPerSecond,
            robotPose.getRotation()
        );
        ShotMotionState motionState = new ShotMotionState(
            robotPose,
            robotRelativeSpeeds,
            fieldSpeeds,
            0.0,
            0.0,
            0.0,
            robotOrientation
        );

        return new RobustShotRequest(
            motionState,
            Translation2d.kZero,
            Constants.Shooter.EXIT_HEIGHT_METERS,
            new Translation3d(4.0, 0.0, Constants.FieldTargets.HUB_TARGET_HEIGHT_METERS),
            Rotation2d.kZero,
            Constants.Shooter.MIN_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_EXIT_SPEED_MPS,
            0.0
        );
    }
}

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
    void stationaryPassbackIsNotLockedToSeventyDegrees() {
        RobustShotSolution solution = passbackAt(4.0, 0.0, 0.0);
        assertTrue(solution.trajectoryFeasible(), solution.quality().reason());
        assertTrue(solution.recommendedLaunchPitch().getDegrees() < 65.0);
        assertTrue(solution.hoodPositionRot() >= Constants.Shooter.MIN_HOOD_POSITION_ROT - EPSILON);
    }

    @Test
    void passbackSelectsSpeedAfterLateralCompensation() {
        for (double distance : new double[] {2.0, 4.0, 6.0, 8.0}) {
            for (double lateral : new double[] {-2.0, 2.0}) {
                RobustShotSolution solution = passbackAt(distance, 0.0, lateral);
                assertTrue(solution.trajectoryFeasible(), "distance=" + distance + ", lateral=" + lateral
                    + ": " + solution.quality().reason());
                assertPassbackHitsTarget(solution, distance, 0.0, lateral);
            }
        }
    }

    @Test
    void passbackCompensatesApproachingAndRecedingMotion() {
        for (double radial : new double[] {-0.75, 0.75}) {
            RobustShotSolution solution = passbackAt(4.013, radial, 1.5);
            assertTrue(solution.trajectoryFeasible(), solution.quality().reason());
            assertPassbackHitsTarget(solution, 4.013, radial, 1.5);
        }
    }

    @Test
    void unreachablePassbackReportsPhysicalFailureWithoutUnsafeSetpoints() {
        RobustShotSolution solution = passbackAt(12.0, 0.0, 0.0);
        assertFalse(solution.trajectoryFeasible());
        assertEquals("no-feasible-passback-shot", solution.quality().reason());
        assertEquals(CommandSource.NONE, solution.commandSource());
        assertTrue(Double.isNaN(solution.rollerRps()));
        assertEquals("outside-passback-domain", passbackAt(12.1, 0.0, 0.0).quality().reason());
    }

    private RobustShotSolution passbackAt(double distance, double radial, double lateral) {
        ChassisSpeeds speeds = new ChassisSpeeds(radial, lateral, 0.0);
        return RobustShotModel.solve(new RobustShotRequest(
            new ShotMotionState(new Pose2d(), speeds, speeds, 0.0, 0.0, 0.0, new Rotation3d()),
            Translation2d.kZero, Constants.Shooter.EXIT_HEIGHT_METERS,
            new Translation3d(distance, 0.0, Constants.FieldTargets.BACK_TARGET_HEIGHT_METERS),
            Rotation2d.kZero, Constants.Shooter.MIN_HOOD_POSITION_ROT,
            Constants.Shooter.MAX_HOOD_POSITION_ROT, Constants.Shooter.MAX_EXIT_SPEED_MPS,
            0.0, ShotTargetConstraints.passback(Constants.FieldTargets.BACK_TARGET_HEIGHT_METERS)
        )).orElseThrow();
    }

    private void assertPassbackHitsTarget(RobustShotSolution solution, double distance,
        double radial, double lateral) {
        // Independently reconstruct the projectile using the commanded launch vector and inherited velocity.
        double time = distance / (solution.launchVectorField().getX() + radial);
        double height = Constants.Shooter.EXIT_HEIGHT_METERS + solution.launchVectorField().getZ() * time
            - 0.5 * 9.81 * time * time;
        assertEquals(0.0, (solution.launchVectorField().getY() + lateral) * time, EPSILON);
        assertEquals(solution.flightTimeSec(), time, EPSILON);
        assertEquals(Constants.FieldTargets.BACK_TARGET_HEIGHT_METERS, height, 0.24);
        assertTrue(Math.abs(solution.rollerRps()) <= Constants.Shooter.MAX_ROLLER_RPS);
        assertTrue(time > 0.0 && time <= 3.0);
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

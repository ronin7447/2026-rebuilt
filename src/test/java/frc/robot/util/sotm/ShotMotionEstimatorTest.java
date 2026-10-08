package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ShotMotionEstimatorTest {
    private ShotMotionEstimator estimator(double alpha) {
        return new ShotMotionEstimator(alpha, 0.10, 8.0, 16.0);
    }

    private ShotMotionEstimator.Estimate sample(ShotMotionEstimator estimator, double time,
        Rotation2d heading, ChassisSpeeds fieldSpeeds) {
        return estimator.sample(time, new Pose2d(0, 0, heading),
            ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, heading),
            new Rotation3d(0, 0, heading.getRadians()));
    }

    private List<String> blockers(ShotMotionEstimator.Estimate estimate, double now) {
        return estimate.shootingBlockers(now, 0.05, 0.14, 2.0, 2.0, Math.PI / 2, Math.PI);
    }

    @Test
    void constantFieldVelocityDuringRotationHasNoFalseTranslationAcceleration() {
        var estimator = estimator(1);
        sample(estimator, 1, Rotation2d.kZero, new ChassisSpeeds(1, 0.5, 1));
        var estimate = sample(estimator, 1.02, Rotation2d.fromRadians(0.02), new ChassisSpeeds(1, 0.5, 1));
        assertEquals(0, estimate.motion().fieldAccelXMps2(), 1e-9);
        assertEquals(0, estimate.motion().fieldAccelYMps2(), 1e-9);
        assertTrue(blockers(estimate, 1.02).isEmpty());
    }

    @Test
    void measuredAccelerationUsesSampleTimeAndDuplicateReadPreservesDerivative() {
        var estimator = estimator(1);
        sample(estimator, 1, Rotation2d.kZero, new ChassisSpeeds());
        var estimate = sample(estimator, 1.02, Rotation2d.kZero, new ChassisSpeeds(0.02, -0.04, 0.02));
        assertEquals(1, estimate.motion().fieldAccelXMps2(), 1e-9);
        assertEquals(-2, estimate.motion().fieldAccelYMps2(), 1e-9);
        assertEquals(1, estimate.motion().angularAccelRadPerSec2(), 1e-9);
        assertSame(estimate, sample(estimator, 1.02, Rotation2d.kZero, new ChassisSpeeds(0.02, -0.04, 0.02)));
    }

    @Test
    void startupGapsTimeReversalNonfiniteAndStaleSamplesBlockFeed() {
        var estimator = estimator(1);
        assertTrue(blockers(sample(estimator, 1, Rotation2d.kZero, new ChassisSpeeds()), 1).contains("invalid-shot-motion"));
        var good = sample(estimator, 1.02, Rotation2d.kZero, new ChassisSpeeds());
        assertTrue(blockers(good, 1.071).contains("invalid-shot-motion"));
        assertTrue(blockers(good, 1.01).contains("invalid-shot-motion"));
        assertTrue(blockers(sample(estimator, 2, Rotation2d.kZero, new ChassisSpeeds()), 2).contains("invalid-shot-motion"));
        assertTrue(blockers(sample(estimator, 1.9, Rotation2d.kZero, new ChassisSpeeds()), 1.9).contains("invalid-shot-motion"));
        var invalid = sample(estimator, 2.02, Rotation2d.kZero, new ChassisSpeeds(Double.NaN, 0, 0));
        assertNull(invalid.motion());
        assertTrue(blockers(invalid, 2.02).contains("invalid-shot-motion"));
    }

    @Test
    void rawAccelerationCannotBeHiddenByClampingOrFiltering() {
        var estimator = estimator(0.25);
        sample(estimator, 1, Rotation2d.kZero, new ChassisSpeeds());
        var estimate = sample(estimator, 1.02, Rotation2d.kZero, new ChassisSpeeds(0.4, 0.4, 1));
        assertEquals(2, estimate.motion().fieldAccelXMps2(), 1e-9);
        assertTrue(blockers(estimate, 1.02).contains("robot-acceleration"));
        assertTrue(blockers(estimate, 1.02).contains("robot-angular-acceleration"));
    }

    @Test
    void predictedReleaseSpeedAndVectorSpeedAreChecked() {
        var estimator = estimator(1);
        sample(estimator, 1, Rotation2d.kZero, new ChassisSpeeds(1.8, 0, 1.5));
        var estimate = sample(estimator, 1.02, Rotation2d.kZero, new ChassisSpeeds(1.84, 0, 1.52));
        var reasons = blockers(estimate, 1.02);
        assertTrue(reasons.contains("robot-speed"));
        assertTrue(reasons.contains("robot-angular-speed"));
        sample(estimator, 2, Rotation2d.kZero, new ChassisSpeeds(1.5, 1.5, 0));
        assertTrue(blockers(sample(estimator, 2.02, Rotation2d.kZero,
            new ChassisSpeeds(1.5, 1.5, 0)), 2.02).contains("robot-speed"));
    }
}

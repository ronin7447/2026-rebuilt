package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ShooterKinematicsTest {
    private static final double EPSILON = 1e-6;
    private static final ShooterCalibration CALIBRATION = new LinearShooterCalibration();
    private static final double MIN_HOOD_POSITION_ROT = -10.0;
    private static final double MAX_HOOD_POSITION_ROT = 10.0;
    private static final double FAR_SHOT_DISTANCE_METERS = 4.0;
    private static final double FAR_SHOT_MIN_HOOD_POSITION_ROT = -1.40;

    @Test
    void zeroVelocityMatchesStationaryAimYaw() {
        Pose2d robotPose = new Pose2d(1.5, 2.0, Rotation2d.fromDegrees(25.0));
        Translation2d robotToTurretPivot = new Translation2d(0.25, 0.15);
        Translation3d target = new Translation3d(5.5, 3.0, 2.0);
        Rotation2d stationaryYaw = AimMath.computeDesiredTurretAngle(
            robotPose,
            robotToTurretPivot,
            Rotation2d.fromDegrees(5.0),
            new Translation2d(target.getX(), target.getY())
        );

        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                robotPose,
                new ChassisSpeeds(),
                robotToTurretPivot,
                0.5,
                target,
                Rotation2d.fromDegrees(5.0),
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isPresent());
        assertEquals(stationaryYaw.getDegrees(), solution.get().desiredTurretYaw().getDegrees(), EPSILON);
        assertEquals(0.0, solution.get().leadYawDeg(), EPSILON);
    }

    @Test
    void forwardShooterOffsetStaysOnFieldForwardAxis() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(0.0, 0.0, Rotation2d.kZero),
                new ChassisSpeeds(),
                new Translation2d(1.0, 0.0),
                0.5,
                new Translation3d(5.0, 0.0, 2.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isPresent());
        assertEquals(0.0, solution.get().desiredFieldYaw().getDegrees(), EPSILON);
    }

    @Test
    void lateralMotionCreatesOppositeDirectionLead() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(0.0, 0.0, Rotation2d.kZero),
                new ChassisSpeeds(0.0, 1.0, 0.0),
                Translation2d.kZero,
                0.5,
                new Translation3d(6.0, 0.0, 2.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isPresent());
        assertTrue(solution.get().desiredFieldYaw().getDegrees() < 0.0);
        assertTrue(solution.get().desiredTurretYaw().getDegrees() < 0.0);
    }

    @Test
    void unsolvableShotReturnsEmpty() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(),
                new ChassisSpeeds(),
                Translation2d.kZero,
                0.5,
                new Translation3d(8.0, 0.0, 3.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                1.0
            )
        );

        assertTrue(solution.isEmpty());
    }

    @Test
    void invalidTargetHeightReturnsEmpty() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(),
                new ChassisSpeeds(),
                Translation2d.kZero,
                0.5,
                new Translation3d(5.0, 0.0, 0.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isEmpty());
    }

    @Test
    void minimumSpeedSelectionAvoidsHighArcBias() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(),
                new ChassisSpeeds(),
                Translation2d.kZero,
                1.0,
                new Translation3d(4.0, 0.0, 1.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isPresent());
        assertEquals(45.0, solution.get().recommendedLaunchPitch().getDegrees(), 5.0);
        assertFalse(Double.isNaN(solution.get().requiredExitSpeedMps()));
    }

    @Test
    void outOfRangeHoodPositionReturnsEmpty() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(),
                new ChassisSpeeds(),
                Translation2d.kZero,
                1.0,
                new Translation3d(4.0, 0.0, 1.0),
                Rotation2d.kZero,
                CALIBRATION,
                -20.0,
                -15.0,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isEmpty());
    }

    @Test
    void farShotsRespectHigherHoodFloor() {
        Optional<ShooterKinematics.MovingShotSolution> solution = ShooterKinematics.solveMovingShot(
            new ShooterKinematics.MovingShotRequest(
                new Pose2d(),
                new ChassisSpeeds(),
                Translation2d.kZero,
                0.5,
                new Translation3d(6.0, 0.0, 2.0),
                Rotation2d.kZero,
                CALIBRATION,
                MIN_HOOD_POSITION_ROT,
                MAX_HOOD_POSITION_ROT,
                FAR_SHOT_DISTANCE_METERS,
                FAR_SHOT_MIN_HOOD_POSITION_ROT,
                20.0
            )
        );

        assertTrue(solution.isPresent());
        double hoodPosition = CALIBRATION.hoodPositionForLaunchAngleDeg(
            solution.get().recommendedLaunchPitch().getDegrees()
        ).orElseThrow();
        assertTrue(hoodPosition >= FAR_SHOT_MIN_HOOD_POSITION_ROT - EPSILON);
    }

    @Test
    void flattestSelectionPrefersLowerLaunchAngleThanMinimumSpeed() {
        ShooterKinematics.MovingShotRequest request = new ShooterKinematics.MovingShotRequest(
            new Pose2d(),
            new ChassisSpeeds(),
            Translation2d.kZero,
            1.0,
            new Translation3d(4.0, 0.0, 1.0),
            Rotation2d.kZero,
            CALIBRATION,
            MIN_HOOD_POSITION_ROT,
            MAX_HOOD_POSITION_ROT,
            FAR_SHOT_DISTANCE_METERS,
            MIN_HOOD_POSITION_ROT,
            20.0
        );

        ShooterKinematics.MovingShotSolution minimumSpeedSolution =
            ShooterKinematics.solveMovingShot(request).orElseThrow();
        ShooterKinematics.MovingShotSolution flattestSolution = ShooterKinematics.solveMovingShot(
            request,
            ShooterKinematics.SelectionPreference.FLATTEST_SHOT
        ).orElseThrow();

        assertTrue(
            flattestSolution.recommendedLaunchPitch().getDegrees()
                < minimumSpeedSolution.recommendedLaunchPitch().getDegrees()
        );
        assertTrue(flattestSolution.requiredExitSpeedMps() >= minimumSpeedSolution.requiredExitSpeedMps());
    }
}

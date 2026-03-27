package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public final class ShooterKinematics {
    private static final double GRAVITY_MPS2 = 9.81;
    private static final double MIN_FLIGHT_TIME_SEC = 0.05;
    private static final double MAX_FLIGHT_TIME_SEC = 3.0;
    private static final double FLIGHT_TIME_STEP_SEC = 0.005;
    private static final double EPSILON = 1e-9;

    private ShooterKinematics() {}

    public static Optional<MovingShotSolution> solveMovingShot(MovingShotRequest request) {
        if (request == null
            || request.targetFieldPoint() == null
            || request.robotPose() == null
            || request.robotRelativeSpeeds() == null
            || request.robotToTurretPivot() == null
            || request.turretZeroOffset() == null) {
            return Optional.empty();
        }

        if (request.targetFieldPoint().getZ() <= 0.0
            || request.shooterExitHeightMeters() < 0.0
            || request.maxExitSpeedMps() <= 0.0
            || request.maxLaunchAngleDeg() < request.minLaunchAngleDeg()) {
            return Optional.empty();
        }

        Translation2d shooterFieldPosition = AimMath.computeTurretPivotFieldPosition(
            request.robotPose(),
            request.robotToTurretPivot()
        );
        Translation3d shooterFieldPoint = new Translation3d(
            shooterFieldPosition.getX(),
            shooterFieldPosition.getY(),
            request.shooterExitHeightMeters()
        );
        Translation3d delta = request.targetFieldPoint().minus(shooterFieldPoint);
        Rotation2d stationaryFieldYaw = AimMath.computeDesiredFieldYaw(
            shooterFieldPosition,
            new Translation2d(request.targetFieldPoint().getX(), request.targetFieldPoint().getY())
        );

        ChassisSpeeds shooterRelativeSpeeds = computeShooterRelativeSpeeds(
            request.robotRelativeSpeeds(),
            request.robotToTurretPivot()
        );
        ChassisSpeeds shooterFieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(
            shooterRelativeSpeeds.vxMetersPerSecond,
            shooterRelativeSpeeds.vyMetersPerSecond,
            shooterRelativeSpeeds.omegaRadiansPerSecond,
            request.robotPose().getRotation()
        );

        MovingShotSolution bestSolution = null;

        for (double flightTimeSec = MIN_FLIGHT_TIME_SEC;
            flightTimeSec <= MAX_FLIGHT_TIME_SEC + EPSILON;
            flightTimeSec += FLIGHT_TIME_STEP_SEC) {
            Translation2d requiredHorizontalVelocity = new Translation2d(
                (delta.getX() / flightTimeSec) - shooterFieldSpeeds.vxMetersPerSecond,
                (delta.getY() / flightTimeSec) - shooterFieldSpeeds.vyMetersPerSecond
            );
            double requiredVerticalVelocity = (delta.getZ() / flightTimeSec) + (0.5 * GRAVITY_MPS2 * flightTimeSec);
            double horizontalSpeed = requiredHorizontalVelocity.getNorm();
            double exitSpeed = Math.hypot(horizontalSpeed, requiredVerticalVelocity);
            double launchPitchDeg = Math.toDegrees(Math.atan2(requiredVerticalVelocity, horizontalSpeed));

            if (exitSpeed > request.maxExitSpeedMps()
                || launchPitchDeg < request.minLaunchAngleDeg()
                || launchPitchDeg > request.maxLaunchAngleDeg()) {
                continue;
            }

            Rotation2d desiredFieldYaw = horizontalSpeed > EPSILON
                ? new Rotation2d(requiredHorizontalVelocity.getX(), requiredHorizontalVelocity.getY())
                : stationaryFieldYaw;
            Rotation2d desiredTurretYaw = AimMath.computeTurretRelativeYaw(
                desiredFieldYaw,
                request.robotPose().getRotation(),
                request.turretZeroOffset()
            );
            MovingShotSolution candidate = new MovingShotSolution(
                desiredFieldYaw,
                desiredTurretYaw,
                Rotation2d.fromDegrees(launchPitchDeg),
                exitSpeed,
                flightTimeSec,
                desiredFieldYaw.minus(stationaryFieldYaw).getDegrees()
            );

            if (bestSolution == null
                || candidate.requiredExitSpeedMps() + EPSILON < bestSolution.requiredExitSpeedMps()
                || (Math.abs(candidate.requiredExitSpeedMps() - bestSolution.requiredExitSpeedMps()) <= EPSILON
                    && candidate.flightTimeSec() + EPSILON < bestSolution.flightTimeSec())) {
                bestSolution = candidate;
            }
        }

        return Optional.ofNullable(bestSolution);
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

    public record MovingShotRequest(
        Pose2d robotPose,
        ChassisSpeeds robotRelativeSpeeds,
        Translation2d robotToTurretPivot,
        double shooterExitHeightMeters,
        Translation3d targetFieldPoint,
        Rotation2d turretZeroOffset,
        double minLaunchAngleDeg,
        double maxLaunchAngleDeg,
        double maxExitSpeedMps
    ) {}

    public record MovingShotSolution(
        Rotation2d desiredFieldYaw,
        Rotation2d desiredTurretYaw,
        Rotation2d recommendedLaunchPitch,
        double requiredExitSpeedMps,
        double flightTimeSec,
        double leadYawDeg
    ) {}
}

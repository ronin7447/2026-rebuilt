package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record RobustShotRequest(
    ShotMotionState motionState,
    Translation2d robotToTurretPivot,
    double shooterExitHeightMeters,
    Translation3d targetFieldPoint,
    Rotation2d turretZeroOffset,
    double minHoodPositionRot,
    double maxHoodPositionRot,
    double maxExitSpeedMps,
    double releaseLatencySec
) {}

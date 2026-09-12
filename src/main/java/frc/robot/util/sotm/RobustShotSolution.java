package frc.robot.util.sotm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;

public record RobustShotSolution(
    Rotation2d desiredFieldYaw,
    Rotation2d desiredTurretYaw,
    Rotation2d recommendedLaunchPitch,
    double hoodPositionRot,
    double rollerRps,
    double requiredExitSpeedMps,
    double flightTimeSec,
    double leadYawDeg,
    Translation3d launchVectorField,
    ShotQuality quality,
    ShotTargetConstraints.SelectionMode selectionMode,
    CommandSource commandSource,
    boolean trajectoryFeasible,
    String trajectoryReason,
    double targetPlaneCrossingTimeSec,
    double solverDurationSec
) {}

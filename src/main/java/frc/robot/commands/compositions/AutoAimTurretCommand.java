package frc.robot.commands.compositions;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.AimMath;
import frc.robot.util.PolygonUtil;

public class AutoAimTurretCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final VisionSubsystem vision;
    private final TurretSubsystem turret;

    public AutoAimTurretCommand(
        CommandSwerveDrivetrain drivetrain,
        VisionSubsystem vision,
        TurretSubsystem turret
    ) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Translation2d pivotFieldPosition = AimMath.computeTurretPivotFieldPosition(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT
        );
        boolean poseFresh = vision.getVisionAgeSec() <= Constants.Turret.MAX_VISION_AGE_FOR_SHOT_SEC;
        boolean visionHealthy = vision.isVisionHealthy();
        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();

        if (!DriverStation.isTeleopEnabled() || turret.isAutoAimSuppressed()) {
            turret.holdCurrentPosition();
            turret.updateAimState(
                allianceOpt.map(Enum::name).orElse("Unknown"),
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                Translation2d.kZero,
                turret.getTurretGoal()
            );
            return;
        }

        if (allianceOpt.isEmpty()) {
            DriverStation.reportError("notWorking", false);
            turret.park();
            turret.updateAimState(
                "Unknown",
                false,
                visionHealthy,
                poseFresh,
                false,
                pivotFieldPosition,
                Translation2d.kZero,
                Constants.Turret.PARK_ANGLE
            );
            return;
        }

        DriverStation.Alliance alliance = allianceOpt.get();
        Translation2d hubPoint = Constants.FieldTargets.getHubForAlliance(alliance);
        boolean inAllianceZone = PolygonUtil.contains(
            Constants.FieldTargets.getAllianceZone(alliance),
            robotPose.getTranslation()
        );
        Rotation2d desiredTurretAngle = AimMath.computeDesiredTurretAngle(
            robotPose,
            Constants.Turret.ROBOT_TO_TURRET_PIVOT,
            Constants.Turret.TURRET_ZERO_OFFSET,
            hubPoint
        );

        boolean autoAimEnabled = inAllianceZone && visionHealthy && poseFresh;
        if (autoAimEnabled) {
            turret.setTurretGoal(desiredTurretAngle);
        } else {
            turret.holdCurrentPosition();
        }

        turret.updateAimState(
            alliance.name(),
            inAllianceZone,
            visionHealthy,
            poseFresh,
            autoAimEnabled,
            pivotFieldPosition,
            hubPoint,
            desiredTurretAngle
        );
    }

    @Override
    public void end(boolean interrupted) {
        turret.holdCurrentPosition();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}

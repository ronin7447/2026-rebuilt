package frc.robot.commands.autopilot;

import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.autopilot.FieldDestination;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Autopilot extends Command {
    private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        3.5,                         // max linear vel m/s
        4.5,                         // max linear accel m/s^2
        Units.degreesToRadians(540), // max angular vel
        Units.degreesToRadians(720)  // max angular accel
    );

    private final CommandSwerveDrivetrain drivetrain;

    public Autopilot(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    public Command goTo(FieldDestination destination) {
        Pose2d goal = switch (destination) {
            case RED_LEFT -> FieldDestinationConstants.RED_LEFT;
            case RED_RIGHT -> FieldDestinationConstants.RED_RIGHT;
            case BLUE_LEFT -> FieldDestinationConstants.BLUE_LEFT;
            case BLUE_RIGHT -> FieldDestinationConstants.BLUE_RIGHT;
            case NEUTRAL_LEFT -> FieldDestinationConstants.NEUTRAL_LEFT;
            case NEUTRAL_RIGHT -> FieldDestinationConstants.NEUTRAL_RIGHT;
        };
        return goTo(goal);
    }

    public Command goTo(Pose2d goal) {
        return Commands.defer(() -> {
            Pose2d currentPose = drivetrain.getState().Pose;

            if (!AutoBuilder.isPathfindingConfigured()) {
                SmartDashboard.putString("Autopilot/Mode", "ERROR");
                DriverStation.reportError(
                    "Autopilot pathfinding is not configured. Check PathPlanner settings/autobuilder init.",
                    false
                );
                return Commands.none();
            }

            publishGoalTelemetry(goal, currentPose);
            if (shouldUseShortRangeAlign(goal, currentPose)) {
                SmartDashboard.putString("Autopilot/Mode", "SHORT_ALIGN");
                return createShortRangeAlignCommand(goal);
            }

            SmartDashboard.putString("Autopilot/Mode", "PATHFIND");
            SmartDashboard.putBoolean("Autopilot/AtPositionTolerance", false);
            SmartDashboard.putBoolean("Autopilot/AtHeadingTolerance", false);
            return createPathfindCommand(goal);
        }, Set.of(drivetrain));
    }

    /** Fixed-field bump-exit move, usable as a sequential PathPlanner named command. */
    public Command goToBumpExit() {
        // Deferred because NamedCommands are registered before AutoBuilder is configured.
        return Commands.defer(() -> {
            if (!AutoBuilder.isPathfindingConfigured()) {
                SmartDashboard.putString("Autopilot/Mode", "ERROR");
                DriverStation.reportError("AutopilotToBumpExit: PathPlanner pathfinding is not configured.", false);
                return Commands.none();
            }
            Pose2d goal = FieldDestinationConstants.BUMP_EXIT_TARGET;
            PathConstraints constraints = new PathConstraints(
                Constants.Drivebase.BUMP_AUTOPILOT_MAX_SPEED_MPS,
                Constants.Drivebase.BUMP_AUTOPILOT_MAX_ACCEL_MPS2,
                DEFAULT_CONSTRAINTS.maxAngularVelocityRadPerSec(),
                DEFAULT_CONSTRAINTS.maxAngularAccelerationRadPerSecSq());
            publishGoalTelemetry(goal, drivetrain.getState().Pose);
            SmartDashboard.putString("Autopilot/Mode", "BUMP_PATHFIND");
            // pathfindToPose uses this exact pose on either alliance and stops at the goal.
            // Do not switch to short-range alignment with different motion constraints.
            return AutoBuilder.pathfindToPose(goal, constraints, 0.0);
        }, Set.of(drivetrain)).finallyDo(interrupted -> {
            drivetrain.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds()));
            SmartDashboard.putString("Autopilot/Mode", "IDLE");
        });
    }

    static boolean shouldUseShortRangeAlign(Pose2d goal, Pose2d currentPose, double thresholdMeters) {
        return goal.getTranslation().getDistance(currentPose.getTranslation()) <= thresholdMeters;
    }

    private boolean shouldUseShortRangeAlign(Pose2d goal, Pose2d currentPose) {
        return shouldUseShortRangeAlign(goal, currentPose, Constants.Drivebase.SHORT_RANGE_ALIGN_DISTANCE_METERS);
    }

    private Command createPathfindCommand(Pose2d goal) {
        return AutoBuilder.pathfindToPose(goal, DEFAULT_CONSTRAINTS, 0.0)
            .finallyDo(interrupted -> SmartDashboard.putString("Autopilot/Mode", "IDLE"));
    }

    private Command createShortRangeAlignCommand(Pose2d goal) {
        return new ShortRangeAutoAlignCommand(drivetrain, goal);
    }

    private void publishGoalTelemetry(Pose2d goal, Pose2d currentPose) {
        SmartDashboard.putNumber("Autopilot/GoalX", goal.getX());
        SmartDashboard.putNumber("Autopilot/GoalY", goal.getY());
        SmartDashboard.putNumber("Autopilot/GoalHeadingDeg", goal.getRotation().getDegrees());
        SmartDashboard.putNumber(
            "Autopilot/DistanceToGoal",
            currentPose.getTranslation().getDistance(goal.getTranslation())
        );
    }
}

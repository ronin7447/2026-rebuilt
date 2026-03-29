package frc.robot.commands.autopilot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.autopilot.FieldDestination;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;

public class Autopilot extends Command {
    private static final PathConstraints DEFAULT_CONSTRAINTS = new PathConstraints(
        3.5,                         // max linear vel m/s
        4.5,                         // max linear accel m/s^2
        Units.degreesToRadians(540), // max angular vel
        Units.degreesToRadians(720)  // max angular accel
    );
    
    public Autopilot() {}

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
        return AutoBuilder.pathfindToPose(goal, DEFAULT_CONSTRAINTS, 0.0);
    }

}

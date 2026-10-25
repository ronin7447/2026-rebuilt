package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import frc.robot.constants.Constants;
import frc.robot.constants.GeneratedFarShotTrials;

class GeneratedFarShotTrialsTest {
    @Test
    void bothFarMapsClearLipAndEnterOpeningAtStationaryAndMovingInputs() {
        var constraints = ShotTargetConstraints.hub(Constants.FieldTargets.HUB_TARGET_HEIGHT_METERS);
        for (int profile : new int[] {1, 2}) {
            for (double distance : new double[] {5.5, 5.75, 6, 6.5, 7, 7.5, 8}) {
                for (double radial : new double[] {-0.5, 0, 0.5}) {
                    String context = "profile=" + profile + " distance=" + distance + " radial=" + radial;
                    double hood = GeneratedFarShotTrials.hood(profile);
                    double rps = GeneratedFarShotTrials.rollerRps(profile, distance, radial);
                    double speed = (Math.abs(rps) - Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT)
                        / Constants.Shooter.ROLLER_RPS_PER_EXIT_SPEED_SLOPE;
                    double pitch = Math.toRadians(RobustShotModel.launchAngleDegForHoodPosition(hood));
                    double horizontal = speed * Math.cos(pitch) + radial;
                    double vertical = speed * Math.sin(pitch);
                    double time = distance / horizontal;
                    double impactHeight = Constants.Shooter.EXIT_HEIGHT_METERS + vertical * time - 4.905 * time * time;
                    double lipTime = (distance - constraints.hubOpeningDepthMeters() / 2) / horizontal;
                    double lipHeight = Constants.Shooter.EXIT_HEIGHT_METERS + vertical * lipTime - 4.905 * lipTime * lipTime;
                    assertTrue(hood >= Constants.Shooter.MIN_HOOD_POSITION_ROT
                        && hood <= Constants.Shooter.MAX_HOOD_POSITION_ROT, context);
                    assertTrue(speed > 0 && speed <= Constants.Shooter.MAX_EXIT_SPEED_MPS, context);
                    assertTrue(Math.abs(rps) <= Constants.Shooter.MAX_ROLLER_RPS, context);
                    assertTrue(Math.abs(impactHeight - constraints.targetHeightMeters()) <= constraints.goalRadiusMeters(), context);
                    assertTrue(lipHeight >= constraints.hubFrontLipHeightMeters()
                        + constraints.fuelRadiusMeters() + constraints.hubLipClearanceMeters(), context);
                    assertTrue(Math.toDegrees(Math.atan2(9.81 * time - vertical, horizontal))
                        >= constraints.minEntryAngleDeg(), context);
                }
            }
        }
    }
}

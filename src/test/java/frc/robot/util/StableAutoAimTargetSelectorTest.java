package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.constants.Constants;
import frc.robot.util.AutoAimTargeting.TargetMode;

class StableAutoAimTargetSelectorTest {
    private StableAutoAimTargetSelector selector() {
        return new StableAutoAimTargetSelector(0.10, 0.15, 0.10);
    }

    @Test
    void zoneBoundaryJitterDoesNotSwitchUntilCandidateIsContinuouslySelected() {
        var selector = selector();
        double boundary = Constants.FieldTargets.BLUE_HUB_AIM_POINT.getX();
        assertEquals(TargetMode.HUB, selector.select(Alliance.Blue,
            new Translation2d(boundary - 0.01, 4), 0, true).targetMode());
        for (int i = 1; i <= 10; i++) {
            double x = boundary + (i % 2 == 0 ? -0.01 : 0.01);
            assertEquals(TargetMode.HUB, selector.select(Alliance.Blue, new Translation2d(x, 4), i * 0.02, true).targetMode());
        }
        for (int i = 11; i <= 15; i++) {
            assertEquals(TargetMode.HUB, selector.select(Alliance.Blue,
                new Translation2d(boundary + 0.01, 4), i * 0.02, true).targetMode());
        }
        assertEquals(TargetMode.NEUTRAL_BACK, selector.select(Alliance.Blue,
            new Translation2d(boundary + 0.01, 4), 0.32, true).targetMode());
    }

    @Test
    void passbackSwitchRequiresDistanceAdvantageAndConfirmation() {
        var selector = selector();
        double lowY = Constants.FieldTargets.BLUE_BACK_TARGET_LOWER.getY();
        double highY = Constants.FieldTargets.BLUE_BACK_TARGET_UPPER.getY();
        double midpoint = (lowY + highY) / 2;
        assertEquals(lowY, selector.select(Alliance.Blue, new Translation2d(8, midpoint - 0.1), 0, true)
            .targetPoint().getY());
        for (int i = 1; i <= 10; i++) {
            assertEquals(lowY, selector.select(Alliance.Blue, new Translation2d(8, midpoint + 0.05), i * 0.02, true)
                .targetPoint().getY());
        }
        for (int i = 11; i <= 16; i++) selector.select(Alliance.Blue, new Translation2d(8, midpoint + 0.10), i * 0.02, true);
        assertEquals(highY, selector.select(Alliance.Blue, new Translation2d(8, midpoint + 0.10), 0.34, true)
            .targetPoint().getY());
    }

    @Test
    void disableAllianceChangeAndSampleGapReacquireImmediately() {
        var selector = selector();
        selector.select(Alliance.Blue, new Translation2d(1, 4), 0, true);
        assertEquals(TargetMode.NEUTRAL_BACK, selector.select(Alliance.Blue, new Translation2d(8, 4), 0.02, false).targetMode());
        assertEquals(TargetMode.HUB, selector.select(Alliance.Red, new Translation2d(15, 4), 0.04, true).targetMode());
        assertEquals(TargetMode.NEUTRAL_BACK, selector.select(Alliance.Red, new Translation2d(8, 4), 0.20, true).targetMode());
    }
}

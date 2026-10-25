package frc.robot.commands.compositions;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import org.junit.jupiter.api.Test;
import frc.robot.constants.Constants;
import frc.robot.util.sotm.ShotReadinessSnapshot;

class HubFeedPolicyTest {
    private ShotReadinessSnapshot snapshot(boolean speedReady, boolean enabled) {
        return ShotReadinessSnapshot.evaluate(0, enabled, false, 99, true, speedReady,
            true, false, true, false, true, true, 99, .1)
            .withMotionBlockers(List.of("robot-acceleration"));
    }

    @Test
    void speedThresholdEnablesBothOutputsAndLatchesOnSameUpdate() {
        var seq = new FeedSequencer(Constants.ShotMap.FEED_RECOVERY_READY_TIME_SEC,
            Constants.ShotMap.OUTTAKE_DELAY_AFTER_READY_SEC, Constants.ShotMap.INDEXER_DELAY_AFTER_OUTTAKE_SEC, true);
        assertFalse(seq.update(0, snapshot(false, true).forHubFeed(false), false, true).permitted());
        var started = seq.update(.02, snapshot(true, true).forHubFeed(false), false, true);
        assertTrue(started.indexerEnabled());
        assertTrue(started.outtakeEnabled());
        // Immediate speed/quality fluctuation cannot reopen startup staging.
        assertTrue(seq.update(.04, snapshot(false, true).forHubFeed(false), false, true).indexerEnabled());
        assertFalse(seq.update(.06, snapshot(true, false).forHubFeed(false), false, true).indexerEnabled());
        assertTrue(seq.update(.08, snapshot(true, true).forHubFeed(false), false, true).indexerEnabled());
        seq.reset();
        assertFalse(seq.update(.10, snapshot(false, true).forHubFeed(false), false, true).indexerEnabled());
    }

    @Test
    void optionalAlignmentGateStillWorksAndDiagnosticFactsRemainUnmodified() {
        var raw = snapshot(true, true);
        assertFalse(raw.ready());
        assertTrue(raw.forHubFeed(false).ready());
        assertFalse(raw.forHubFeed(false).solutionValid());
        assertFalse(raw.forHubFeed(true).ready());
    }
}

package frc.robot.commands.compositions;

import static org.junit.jupiter.api.Assertions.*;
import java.util.List;
import org.junit.jupiter.api.Test;
import frc.robot.util.sotm.ShotReadinessSnapshot;

class PassbackFeedPolicyTest {
    private ShotReadinessSnapshot snapshot(int hardFault) {
        return ShotReadinessSnapshot.evaluate(1, hardFault != 0, false, 99,
            hardFault != 1, false, hardFault != 2, false, hardFault != 3, false,
            hardFault != 4, hardFault != 5, 99, .1)
            .withMotionBlockers(List.of("invalid-shot-motion", "robot-speed"));
    }

    @Test
    void passbackStartsDespiteQualityBlockersWhileHubStillWaits() {
        var raw = snapshot(-1);
        assertFalse(raw.ready());
        var pass = raw.forPassbackFeed();
        assertTrue(pass.ready());
        assertFalse(pass.solutionValid()); // Diagnostic facts are preserved, not falsified.
        assertFalse(pass.shooterAtTarget());
        var hubSequencer = new FeedSequencer(.05, .05, .025, true);
        assertFalse(hubSequencer.update(1, raw, false, true).permitted());
        var passSequencer = new FeedSequencer(.05, .05, .025, true);
        assertTrue(passSequencer.update(1, pass, false, true).permitted());
        assertFalse(passSequencer.state().indexerEnabled());
        assertTrue(passSequencer.update(1.075, pass, false, true).indexerEnabled());
        assertFalse(hubSequencer.update(1.075, raw, false, true).indexerEnabled());
    }

    @Test
    void passbackRetainsHardGuardsBeforeAndDuringFeed() {
        for (int guard = 0; guard < 6; guard++) {
            var bad = snapshot(guard).forPassbackFeed();
            assertFalse(bad.ready());
            var seq = new FeedSequencer(.05, .05, .025, true);
            assertFalse(seq.update(0, bad, false, true).permitted());
            seq.update(1, snapshot(-1).forPassbackFeed(), false, true);
            assertTrue(seq.update(1.075, snapshot(-1).forPassbackFeed(), false, true).indexerEnabled());
            assertFalse(seq.update(1.1, bad, false, true).indexerEnabled());
        }
    }
}

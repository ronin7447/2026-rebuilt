package frc.robot.commands.compositions;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.Test;
import frc.robot.util.sotm.ShotReadinessSnapshot;

class FeedSequencerTest {
    @Test
    void stagesInitialFeedAndDropsBothOutputsImmediately() {
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        assertFalse(sequencer.update(0.0, ready(), false, true).outtakeEnabled());
        assertTrue(sequencer.update(0.25, ready(), false, true).outtakeEnabled());
        assertTrue(sequencer.update(0.35, ready(), false, true).indexerEnabled());
        FeedSequencer.State lost = sequencer.update(0.36, blocked(), false, true);
        assertFalse(lost.outtakeEnabled());
        assertFalse(lost.indexerEnabled());
    }

    @Test
    void recoveryMustRemainReadyBeforeRestaging() {
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        sequencer.update(0.0, ready(), false, true);
        sequencer.update(0.25, ready(), false, true);
        sequencer.update(0.30, blocked(), false, true);
        assertEquals("recovering-readiness", sequencer.update(0.31, ready(), false, true).reason());
        sequencer.update(0.36, blocked(), false, true);
        assertEquals("recovering-readiness", sequencer.update(0.40, ready(), false, true).reason());
        assertFalse(sequencer.update(0.50, ready(), false, true).outtakeEnabled());
        assertTrue(sequencer.update(0.75, ready(), false, true).outtakeEnabled());
    }

    @Test
    void overrideCannotBypassEnabledNumericOrActuatorProtections() {
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        ShotReadinessSnapshot ordinaryBlock = new ShotReadinessSnapshot(
            0, true, false, 1, true, false, true, false, true, false, true, true,
            Double.POSITIVE_INFINITY, List.of("invalid-shot-solution"));
        assertTrue(sequencer.update(0, ordinaryBlock, true, true).overrideActive());
        ShotReadinessSnapshot unsafe = new ShotReadinessSnapshot(
            0, false, false, 1, true, false, true, false, true, false, true, true,
            Double.POSITIVE_INFINITY, List.of("robot-not-enabled"));
        assertFalse(sequencer.update(1, unsafe, true, true).permitted());
    }

    @Test
    void visionAgeIsWarningOnlyAndInterlockOffRestoresSpeedLatchPolicy() {
        ShotReadinessSnapshot staleVision = ready();
        assertTrue(staleVision.ready());
        assertTrue(staleVision.visionDegraded(0.25));
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        assertTrue(sequencer.update(0, staleVision, false, true).permitted());
        assertTrue(sequencer.update(0, blockedButAtSpeed(), false, false).permitted());
    }

    private static ShotReadinessSnapshot ready() {
        return new ShotReadinessSnapshot(0, true, true, 0, true, true, true, true, true, true, true, true,
            Double.POSITIVE_INFINITY, List.of());
    }
    private static ShotReadinessSnapshot blocked() {
        return new ShotReadinessSnapshot(0, true, false, 0, true, false, true, true, true, true, true, true,
            0, List.of("invalid-shot-solution", "shooter-speed"));
    }
    private static ShotReadinessSnapshot blockedButAtSpeed() {
        return new ShotReadinessSnapshot(0, true, false, 1, true, true, true, false, false, false, true, true,
            0, List.of("invalid-shot-solution"));
    }
}

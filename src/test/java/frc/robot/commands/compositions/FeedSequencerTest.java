package frc.robot.commands.compositions;

import static org.junit.jupiter.api.Assertions.*;

import java.util.List;
import org.junit.jupiter.api.Test;
import frc.robot.util.sotm.ShotReadinessSnapshot;
import frc.robot.constants.Constants;
import frc.robot.util.ShooterSpeedReadiness;

class FeedSequencerTest {
    @Test
    void floatingFlywheelStartsQuicklyAndDoesNotRepeatedlyRestageFeed() {
        var speed = new ShooterSpeedReadiness(Constants.Shooter.ROLLER_START_TOLERANCE_RPS,
            Constants.Shooter.ROLLER_START_TOLERANCE_SCALE,
            Constants.Shooter.ROLLER_RUNNING_TOLERANCE_RPS, Constants.Shooter.ROLLER_RUNNING_TOLERANCE_SCALE);
        var sequencer = new FeedSequencer(Constants.ShotMap.FEED_RECOVERY_READY_TIME_SEC,
            Constants.ShotMap.OUTTAKE_DELAY_AFTER_READY_SEC, Constants.ShotMap.INDEXER_DELAY_AFTER_OUTTAKE_SEC);
        for (int i = 0; i <= 30; i++) {
            double actual = i % 2 == 0 ? 26.5 : 28.5;
            boolean speedReady = speed.update(actual, 25, true);
            assertTrue(speedReady);
            var snapshot = ShotReadinessSnapshot.evaluate(i * 0.02, true, true, 0,
                true, speedReady, true, true, true, true, true, true, 0, 0.10);
            var state = sequencer.update(i * 0.02, snapshot, false, true);
            if (i >= 4) assertTrue(state.indexerEnabled()); // Starts by the 80 ms scheduler sample.
        }
        var droop = ShotReadinessSnapshot.evaluate(0.62, true, true, 0,
            true, speed.update(15, 25, true), true, true, true, true, true, true, 0, 0.10);
        assertFalse(sequencer.update(0.62, droop, false, true).indexerEnabled());
    }

    @Test
    void forceFeedIsImmediateWithoutAutoAimSpeedAlignmentOrMotionReadiness() {
        var sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        var blocked = new ShotReadinessSnapshot(0, true, false, 99, true, false,
            true, false, false, false, false, true, Double.POSITIVE_INFINITY,
            List.of("invalid-shot-solution", "shooter-speed", "turret-alignment", "robot-speed"));
        var forced = sequencer.update(0, blocked, true, true);
        assertTrue(forced.overrideActive());
        assertTrue(forced.outtakeEnabled());
        assertTrue(forced.indexerEnabled());
        assertEquals("manual-override", forced.reason());
        var released = sequencer.update(0.02, blocked, false, true);
        assertFalse(released.outtakeEnabled());
        assertFalse(released.indexerEnabled());
        assertTrue(sequencer.update(0.04, blocked, true, false).indexerEnabled());
    }

    @Test
    void releasingForceFeedRestoresAutomaticRecoveryBeforeStaging() {
        var sequencer = new FeedSequencer(0.05, 0.05, 0.025);
        sequencer.update(0, blocked(), true, true);
        assertEquals("recovering-readiness", sequencer.update(0.02, ready(), false, true).reason());
        assertFalse(sequencer.update(0.07, ready(), false, true).indexerEnabled());
        assertTrue(sequencer.update(0.145, ready(), false, true).indexerEnabled());
    }

    @Test
    void motionLimitDropsFeedAndRequiresContinuousRecoveryBeforeRestaging() {
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        sequencer.update(0, ready(), false, true);
        assertTrue(sequencer.update(0.35, ready(), false, true).indexerEnabled());
        var blocked = ready().withMotionBlockers(List.of("robot-acceleration"));
        var lost = sequencer.update(0.36, blocked, false, true);
        assertFalse(lost.outtakeEnabled());
        assertFalse(lost.indexerEnabled());
        assertEquals("robot-acceleration", lost.reason());
        assertEquals("recovering-readiness", sequencer.update(0.4, ready(), false, true).reason());
        assertFalse(sequencer.update(0.5, ready(), false, true).outtakeEnabled());
        assertTrue(sequencer.update(0.85, ready(), false, true).indexerEnabled());
        // Existing deliberate manual override policy applies to motion readiness as well.
        assertTrue(sequencer.update(0.86, blocked, true, true).overrideActive());
    }

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
    void overrideCannotBypassDisabledRobotDisabledShooterOrInvalidCommands() {
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        ShotReadinessSnapshot ordinaryBlock = new ShotReadinessSnapshot(
            0, true, false, 1, true, false, true, false, true, false, true, true,
            Double.POSITIVE_INFINITY, List.of("invalid-shot-solution"));
        assertTrue(sequencer.update(0, ordinaryBlock, true, true).overrideActive());
        ShotReadinessSnapshot unsafe = new ShotReadinessSnapshot(
            0, false, false, 1, true, false, true, false, true, false, true, true,
            Double.POSITIVE_INFINITY, List.of("robot-not-enabled"));
        assertFalse(sequencer.update(1, unsafe, true, true).permitted());
        var disabledShooter = new ShotReadinessSnapshot(
            0, true, false, 1, false, false, true, false, false, false, true, true,
            0, List.of("shooter-disabled"));
        assertFalse(sequencer.update(2, disabledShooter, true, true).permitted());
        var invalidCommands = new ShotReadinessSnapshot(
            0, true, false, 1, true, false, true, false, false, false, true, false,
            0, List.of("invalid-setpoint"));
        assertFalse(sequencer.update(3, invalidCommands, true, true).permitted());
    }

    @Test
    void visionAgeIsWarningOnly() {
        ShotReadinessSnapshot staleVision = ready();
        assertTrue(staleVision.ready());
        assertTrue(staleVision.visionDegraded(0.25));
        FeedSequencer sequencer = new FeedSequencer(0.10, 0.25, 0.10);
        assertTrue(sequencer.update(0, staleVision, false, true).permitted());
    }

    @Test
    void interlockOffFeedsDespiteSpeedAlignmentSolutionAndMotionReadiness() {
        var sequencer = new FeedSequencer(0.10, 0.05, 0.025);
        var notReady = new ShotReadinessSnapshot(0, true, false, 99, true, false,
            false, false, false, false, false, true, Double.POSITIVE_INFINITY,
            List.of("invalid-shot-solution", "stale-shot-solution", "shooter-speed",
                "hood-position", "turret-alignment", "turret-software-limit", "robot-acceleration"));
        var starting = sequencer.update(0, notReady, false, false);
        assertTrue(starting.permitted());
        assertFalse(starting.outtakeEnabled());
        assertEquals("readiness-bypassed", starting.reason());
        assertTrue(sequencer.update(0.05, notReady, false, false).outtakeEnabled());
        assertTrue(sequencer.update(0.075, notReady, false, false).indexerEnabled());
        assertTrue(sequencer.update(0.10, ready(), false, false).indexerEnabled());
        assertTrue(sequencer.update(0.12, notReady, false, false).indexerEnabled());
        assertFalse(sequencer.state().overrideActive());

        // Re-enabling the checks immediately restores their effect.
        assertFalse(sequencer.update(0.14, notReady, false, true).permitted());
    }

    @Test
    void interlockOffStillStopsForDisabledRobotDisabledShooterAndInvalidSetpoints() {
        for (int failedGuard = 0; failedGuard < 3; failedGuard++) {
            var sequencer = new FeedSequencer(0.10, 0.05, 0.025);
            sequencer.update(0, ready(), false, false);
            assertTrue(sequencer.update(0.075, ready(), false, false).indexerEnabled());
            var unsafe = new ShotReadinessSnapshot(0, failedGuard != 0, true, 0,
                failedGuard != 1, true, true, true, true, true, true, failedGuard != 2,
                0, List.of("basic-feed-protection"));
            var stopped = sequencer.update(0.10, unsafe, false, false);
            assertFalse(stopped.permitted());
            assertFalse(stopped.outtakeEnabled());
            assertFalse(stopped.indexerEnabled());
        }
    }

    @Test
    void releasingOverrideWithInterlockOffDoesNotWaitForReadinessRecovery() {
        var sequencer = new FeedSequencer(0.50, 0.05, 0.025);
        assertTrue(sequencer.update(0, blocked(), true, false).indexerEnabled());
        assertEquals("readiness-bypassed", sequencer.update(0.02, blocked(), false, false).reason());
        assertTrue(sequencer.update(0.095, blocked(), false, false).indexerEnabled());
    }

    private static ShotReadinessSnapshot ready() {
        return new ShotReadinessSnapshot(0, true, true, 0, true, true, true, true, true, true, true, true,
            Double.POSITIVE_INFINITY, List.of());
    }
    private static ShotReadinessSnapshot blocked() {
        return new ShotReadinessSnapshot(0, true, false, 0, true, false, true, true, true, true, true, true,
            0, List.of("invalid-shot-solution", "shooter-speed"));
    }
}

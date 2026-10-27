package frc.robot.commands.intake;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class HomingStateMachineTest {
    @Test
    void onlyConfirmedStallSucceeds() {
        HomingStateMachine state = new HomingStateMachine(3.0);
        state.begin(10.0);
        assertFalse(state.update(10.5, false));
        assertTrue(state.update(10.6, true));
        assertEquals(HomingStateMachine.Outcome.SUCCESS, state.end(false));
    }

    @Test
    void timeoutAndInterruptionAreNotSuccess() {
        HomingStateMachine timeout = new HomingStateMachine(3.0);
        timeout.begin(0.0);
        assertTrue(timeout.update(3.0, false));
        assertEquals(HomingStateMachine.Outcome.TIMEOUT, timeout.end(false));

        HomingStateMachine interrupted = new HomingStateMachine(3.0);
        interrupted.begin(0.0);
        assertEquals(HomingStateMachine.Outcome.INTERRUPTED, interrupted.end(true));
    }

    @Test
    void repeatedAttemptsResetPriorSuccess() {
        HomingStateMachine state = new HomingStateMachine(1.0);
        state.begin(0.0);
        state.update(0.1, true);
        state.begin(2.0);
        assertEquals(HomingStateMachine.Outcome.RUNNING, state.outcome());
        assertFalse(state.update(2.1, false));
    }
}

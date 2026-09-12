package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import java.util.LinkedHashMap;
import java.util.Map;
import org.junit.jupiter.api.Test;

class PowerHealthRecorderTest {
    @Test
    void absentPdhIsVisibleWithoutBlockingSampling() {
        CapturingSink sink = new CapturingSink();
        PowerHealthRecorder recorder = new PowerHealthRecorder(
            () -> 1.0,
            sink,
            PowerHealthRecorder.PowerSource.unavailable(),
            () -> new PowerHealthRecorder.Context(false, "disabled", false, false, false, 0)
        );
        recorder.periodic();
        assertEquals(false, sink.booleans.get("Power/PDHAvailable"));
        assertTrue(recorder.isHealthy());
    }

    @Test
    void storageFailureIsContainedAndReported() {
        PowerHealthRecorder.SampleSink failing = new PowerHealthRecorder.SampleSink() {
            public void number(String name, double value) { throw new IllegalStateException("disk-full"); }
            public void bool(String name, boolean value) {}
            public void text(String name, String value) {}
            public boolean available() { return true; }
        };
        PowerHealthRecorder recorder = new PowerHealthRecorder(
            () -> 2.0,
            failing,
            PowerHealthRecorder.PowerSource.unavailable(),
            () -> new PowerHealthRecorder.Context(true, "teleop", true, false, false, -50)
        );
        assertDoesNotThrow(recorder::periodic);
        assertFalse(recorder.isHealthy());
        assertTrue(recorder.healthReason().contains("disk-full"));
    }

    @Test
    void staleAndInvalidSampleAgesAreNeverFresh() {
        assertTrue(PowerHealthRecorder.isFreshSample(true, true, 0.02, 0.05));
        assertFalse(PowerHealthRecorder.isFreshSample(false, true, 0.02, 0.05));
        assertFalse(PowerHealthRecorder.isFreshSample(true, false, 0.02, 0.05));
        assertFalse(PowerHealthRecorder.isFreshSample(true, true, 0.06, 0.05));
        assertFalse(PowerHealthRecorder.isFreshSample(true, true, Double.NaN, 0.05));
    }

    @Test
    void unknownPdhChannelsKeepStableNumericLabels() {
        assertEquals("Power/PDHChannel/0/CurrentA", PowerHealthRecorder.pdhChannelCurrentKey(0));
        assertEquals("Power/PDHChannel/23/CurrentA", PowerHealthRecorder.pdhChannelCurrentKey(23));
        assertThrows(IllegalArgumentException.class, () -> PowerHealthRecorder.pdhChannelCurrentKey(-1));
    }

    private static final class CapturingSink implements PowerHealthRecorder.SampleSink {
        final Map<String, Double> numbers = new LinkedHashMap<>();
        final Map<String, Boolean> booleans = new LinkedHashMap<>();
        public void number(String name, double value) { numbers.put(name, value); }
        public void bool(String name, boolean value) { booleans.put(name, value); }
        public void text(String name, String value) {}
        public boolean available() { return true; }
    }
}

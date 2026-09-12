package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class VisionTimestampPolicyTest {
    @Test
    void rejectsNonfiniteFutureAndOldCaptureTimes() {
        assertFalse(VisionTimestampPolicy.isValid(10, Double.NaN, 0.05, 1.0));
        assertFalse(VisionTimestampPolicy.isValid(10, 10.1, 0.05, 1.0));
        assertFalse(VisionTimestampPolicy.isValid(10, 8.9, 0.05, 1.0));
        assertTrue(VisionTimestampPolicy.isValid(10, 9.5, 0.05, 1.0));
    }

    @Test
    void acceptedTimeNeverMovesBackwardAcrossCameras() {
        assertEquals(9.8, VisionTimestampPolicy.monotonicAcceptedTimestamp(9.8, 9.7), 1e-9);
        assertEquals(9.9, VisionTimestampPolicy.monotonicAcceptedTimestamp(9.8, 9.9), 1e-9);
    }
}

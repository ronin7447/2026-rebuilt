package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import java.util.concurrent.atomic.AtomicInteger;
import com.ctre.phoenix6.StatusCode;
import org.junit.jupiter.api.Test;

class ConfigurationHealthTest {
    @Test
    void retriesAreBoundedAndSuccessIsRecordedSeparatelyFromRequest() {
        AtomicInteger attempts = new AtomicInteger();
        boolean applied = ConfigurationHealth.apply("test-eventual-success", () ->
            attempts.incrementAndGet() == 2 ? StatusCode.OK : StatusCode.TaskIsBusy);
        assertTrue(applied);
        assertEquals(2, attempts.get());
        ConfigurationHealth.Result result = ConfigurationHealth.snapshot().get("test-eventual-success");
        assertTrue(result.requested());
        assertTrue(result.applied());
        assertEquals(2, result.attempts());
    }

    @Test
    void stopsAfterConfiguredMaximumAttempts() {
        AtomicInteger attempts = new AtomicInteger();
        assertFalse(ConfigurationHealth.apply("test-always-fails", () -> {
            attempts.incrementAndGet();
            return StatusCode.ConfigFailed;
        }));
        assertEquals(3, attempts.get());
        assertFalse(ConfigurationHealth.snapshot().get("test-always-fails").applied());
    }
}

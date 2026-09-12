package frc.robot.util;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;

/** Bounded startup configuration application with persistent, named health state. */
public final class ConfigurationHealth {
    private static final Map<String, Result> RESULTS = new LinkedHashMap<>();

    private ConfigurationHealth() {}

    public static boolean apply(String deviceName, Supplier<StatusCode> attempt) {
        StatusCode lastStatus = StatusCode.GeneralError;
        int attempts = 0;
        for (; attempts < Constants.PowerLogging.CONFIG_APPLY_ATTEMPTS; attempts++) {
            lastStatus = attempt.get();
            if (lastStatus.isOK()) break;
        }
        boolean success = lastStatus.isOK();
        Result result = new Result(deviceName, true, success, attempts + (success ? 1 : 0), lastStatus.getName());
        synchronized (RESULTS) {
            RESULTS.put(deviceName, result);
        }
        SmartDashboard.putBoolean("Health/Config/" + deviceName + "/Applied", success);
        SmartDashboard.putString("Health/Config/" + deviceName + "/Status", lastStatus.getName());
        if (!success) {
            DriverStation.reportError(
                "Essential configuration failed for " + deviceName + ": " + lastStatus.getName(),
                false
            );
        }
        return success;
    }

    public static Map<String, Result> snapshot() {
        synchronized (RESULTS) {
            return Map.copyOf(RESULTS);
        }
    }

    public record Result(
        String deviceName,
        boolean requested,
        boolean applied,
        int attempts,
        String status
    ) {}
}

package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class TurretDiagnosticsTest {
    @Test
    void separatesOvershootBiasAndProfileLag() {
        TurretDiagnostics metrics = new TurretDiagnostics(0.5, 0.1, 0.1, 1.0);
        metrics.beginStep(0.0, 0.0, 10.0);
        metrics.update(0.05, 10.0, 4.0, 2.0);
        metrics.update(0.10, 10.0, 9.0, 11.0);
        TurretDiagnostics.Snapshot snapshot = metrics.update(0.20, 10.0, 10.0, 9.8);
        assertEquals(1.0, snapshot.overshootDeg(), 1e-9);
        assertEquals(0.2, snapshot.finalBiasDeg(), 1e-9);
        assertTrue(snapshot.errorSignChanges() >= 1);
        assertNotEquals(snapshot.rollingGoalErrorRmsDeg(), snapshot.rollingProfileErrorRmsDeg());
    }

    @Test
    void continuousToleranceIsRequiredBeforeSettled() {
        TurretDiagnostics metrics = new TurretDiagnostics(0.5, 0.1, 0.1, 1.0);
        metrics.beginStep(0.0, 0.0, 5.0);
        assertTrue(Double.isNaN(metrics.update(0.05, 5, 5, 4.8).settlingTimeSec()));
        metrics.update(0.10, 5, 5, 4.4);
        metrics.update(0.20, 5, 5, 4.8);
        assertTrue(Double.isNaN(metrics.update(0.25, 5, 5, 4.8).settlingTimeSec()));
        assertEquals(0.30, metrics.update(0.30, 5, 5, 4.8).settlingTimeSec(), 1e-9);
    }

    @Test
    void resetClearsStepAndRollingWindow() {
        TurretDiagnostics metrics = new TurretDiagnostics(0.5, 0.1, 0.1, 1.0);
        metrics.beginStep(0.0, 0.0, 5.0);
        metrics.update(0.1, 5, 4, 3);
        metrics.resetAll();
        TurretDiagnostics.Snapshot snapshot = metrics.update(2.0, 0, 0, 0);
        assertFalse(snapshot.stationaryStepActive());
        assertEquals(1, snapshot.rollingSampleCount());
        assertEquals(0, snapshot.overshootDeg(), 1e-9);
    }
}

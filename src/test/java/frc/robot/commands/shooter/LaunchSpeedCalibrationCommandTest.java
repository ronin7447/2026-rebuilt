package frc.robot.commands.shooter;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

class LaunchSpeedCalibrationCommandTest {
    @BeforeAll
    static void initializeHal() {
        // Initialize FPGA time before publishing any NetworkTables values.
        assertTrue(HAL.initialize(500, 0));
    }

    @Test
    void sharedInterlockSwitchBypassesCalibrationMechanismReadiness() {
        assertEquals("", LaunchSpeedCalibrationCommand.readinessBlocker(false, false, false, false));
        assertEquals("waiting-for-shooter-speed",
            LaunchSpeedCalibrationCommand.readinessBlocker(true, false, false, false));
        assertEquals("waiting-for-hood",
            LaunchSpeedCalibrationCommand.readinessBlocker(true, true, false, false));
        assertEquals("waiting-for-turret",
            LaunchSpeedCalibrationCommand.readinessBlocker(true, true, true, false));
        assertEquals("", LaunchSpeedCalibrationCommand.readinessBlocker(true, true, true, true));
    }

    @Test
    void usesMotorRpsAndExistingSeventyDegreeHoodMapping() {
        FakeMechanisms mechanisms = new FakeMechanisms();
        var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> 25.0);
        command.initialize();

        assertEquals(-25.0, mechanisms.signedRps);
        assertEquals(-1.489029, mechanisms.hoodRot, 1e-6);
        assertTrue(mechanisms.running);
        assertFalse(mechanisms.feeding);
        assertFalse(command.isFinished());
    }

    @Test
    void samplesOncePerTrialAndAcceptsNewValueAfterStopAndRestart() {
        FakeMechanisms mechanisms = new FakeMechanisms();
        double[] requested = {25.0};
        var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> requested[0]);
        command.initialize();
        requested[0] = 2000.0 / 60.0;
        command.execute();
        assertEquals(-25.0, mechanisms.signedRps);

        command.end(true);
        command.initialize();
        assertEquals(-2000.0 / 60.0, mechanisms.signedRps);
        assertFalse(mechanisms.feeding);
    }

    @Test
    void neverFeedsFromElapsedCyclesAndRespondsImmediatelyToEveryReadinessChange() {
        FakeMechanisms mechanisms = new FakeMechanisms();
        var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> 25.0);
        command.initialize();
        for (int cycle = 0; cycle < 1000; cycle++) {
            command.execute();
            assertFalse(mechanisms.feeding);
        }

        for (String blocker : new String[] {
            "waiting-for-shooter-speed", "waiting-for-hood", "waiting-for-turret",
            "invalid-setpoints", "not-teleop-enabled"
        }) {
            mechanisms.blocker = "";
            command.execute();
            assertTrue(mechanisms.feeding, "Ready feedback should feed immediately");
            mechanisms.blocker = blocker;
            command.execute();
            assertFalse(mechanisms.feeding, blocker);
        }
        mechanisms.blocker = "";
        command.execute();
        assertTrue(mechanisms.feeding, "Recovery does not use a fixed delay");
    }

    @Test
    void rejectsInvalidOrRpmSizedInputsWithoutStartingAnyMotor() {
        for (double invalid : new double[] {
            0.0, -25.0, 100.01, 1500.0, 2000.0, Double.NaN,
            Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY
        }) {
            FakeMechanisms mechanisms = new FakeMechanisms();
            mechanisms.blocker = "";
            var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> invalid);
            command.initialize();
            command.execute();
            assertTrue(command.isFinished());
            assertFalse(mechanisms.running);
            assertFalse(mechanisms.feeding);
            assertEquals(0, mechanisms.starts);
        }
        FakeMechanisms mechanisms = new FakeMechanisms();
        var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> Constants.Shooter.MAX_ROLLER_RPS);
        command.initialize();
        assertTrue(mechanisms.running);
    }

    @Test
    void stopShutsDownDuringSpinupOrFeeding() {
        for (String blocker : new String[] {"waiting-for-shooter-speed", ""}) {
            FakeMechanisms mechanisms = new FakeMechanisms();
            mechanisms.blocker = blocker;
            var command = new LaunchSpeedCalibrationCommand(mechanisms, () -> 25.0);
            command.initialize();
            command.execute();
            command.end(true);
            assertFalse(mechanisms.running);
            assertFalse(mechanisms.feeding);
            assertFalse(SmartDashboard.getBoolean(LaunchSpeedCalibrationCommand.DASHBOARD_PREFIX + "Running", true));
        }
    }

    @Test
    void dashboardSwitchSelectsDashboardOrCodeWithoutConvertingUnits() {
        String prefix = LaunchSpeedCalibrationCommand.DASHBOARD_PREFIX;
        SmartDashboard.putNumber(prefix + "RequestedRPS", 2000.0 / 60.0);
        SmartDashboard.putBoolean(prefix + "UseDashboardRPS", true);
        assertEquals(2000.0 / 60.0, LaunchSpeedCalibrationCommand.readRequestedSpeedRps());
        SmartDashboard.putBoolean(prefix + "UseDashboardRPS", false);
        assertEquals(Constants.LaunchSpeedCalibration.ROLLER_SPEED_RPS,
            LaunchSpeedCalibrationCommand.readRequestedSpeedRps());
    }

    @Test
    void schedulerCancelsConflictingAimAndStopsTrialOnDisable() {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        SubsystemBase mechanismSubsystem = new SubsystemBase() {};
        FakeMechanisms mechanisms = new FakeMechanisms();
        mechanisms.blocker = "";
        try {
            DriverStationSim.setDsAttached(true);
            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            var aim = mechanismSubsystem.run(() -> {});
            scheduler.schedule(aim);
            assertTrue(aim.isScheduled());
            var calibration = new LaunchSpeedCalibrationCommand(mechanisms, () -> 25.0, mechanismSubsystem);
            scheduler.schedule(calibration);
            assertFalse(aim.isScheduled());
            assertTrue(calibration.isScheduled());
            scheduler.run();
            assertTrue(mechanisms.feeding);

            DriverStationSim.setEnabled(false);
            DriverStationSim.notifyNewData();
            scheduler.run();
            assertFalse(calibration.isScheduled());
            assertFalse(mechanisms.running);
            assertFalse(mechanisms.feeding);

            DriverStationSim.setEnabled(true);
            DriverStationSim.notifyNewData();
            scheduler.run();
            assertFalse(calibration.isScheduled(), "Re-enabling requires a new A press");
        } finally {
            scheduler.cancelAll();
            scheduler.unregisterSubsystem(mechanismSubsystem);
            DriverStationSim.resetData();
            DriverStationSim.notifyNewData();
        }
    }

    private static final class FakeMechanisms implements LaunchSpeedCalibrationCommand.Mechanisms {
        double signedRps;
        double hoodRot;
        int starts;
        boolean running;
        boolean feeding;
        String blocker = "waiting-for-shooter-speed";

        public void start(double signedRps, double hoodRot) {
            this.signedRps = signedRps;
            this.hoodRot = hoodRot;
            starts++;
            running = true;
        }

        public String feedBlocker() { return blocker; }
        public void setFeed(boolean enabled) { feeding = enabled; }
        public double actualRps() { return signedRps; }
        public void stop() {
            running = false;
            feeding = false;
        }
    }
}

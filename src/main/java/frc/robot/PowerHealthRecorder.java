package frc.robot;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.TunerConstants;
import frc.robot.util.ConfigurationHealth;

/** Single owner for synchronized power, motor, CAN, and robot-health recording. */
public final class PowerHealthRecorder {
    private static boolean logStartAttempted;
    private static boolean dataLogAvailable;
    private final DoubleSupplier clockSec;
    private final SampleSink sink;
    private final PowerSource powerSource;
    private final List<MotorSignals> motors = new ArrayList<>();
    private final CANBus canivore = TunerConstants.kCANBus;
    private final Supplier<Context> contextSupplier;
    private final boolean platformHealthEnabled;
    private double nextSlowSampleSec;
    private double nextDashboardSec;
    private volatile boolean healthy = true;
    private volatile String healthReason = "ready";
    private MotorSignals turretSignals;
    private Notifier turretMotionNotifier;

    public static PowerHealthRecorder create(RobotContainer container) {
        startDataLogEarly();
        SampleSink sink = dataLogAvailable ? new WpiDataLogSink() : SampleSink.disabled();

        PowerSource powerSource = PowerSource.unavailable();
        if (RobotBase.isReal()) {
            try {
                PowerDistribution pdh = Constants.PowerLogging.REV_PDH_CAN_ID < 0
                    ? new PowerDistribution()
                    : new PowerDistribution(Constants.PowerLogging.REV_PDH_CAN_ID, PowerDistribution.ModuleType.kRev);
                powerSource = PowerSource.from(pdh);
            } catch (RuntimeException exception) {
                DriverStation.reportWarning("REV PDH unavailable for logging: " + exception.getMessage(), false);
            }
        }

        PowerHealthRecorder recorder = new PowerHealthRecorder(
            Timer::getFPGATimestamp,
            sink,
            powerSource,
            container::recorderContext,
            true
        );
        try {
            recorder.register("intake-extension-master", container.m_IntakeSubsystem.getExtensionKrakenMaster(), "CANivore1");
            recorder.register("intake-extension-follower", container.m_IntakeSubsystem.getExtensionKrakenFollower(), "CANivore1");
            recorder.register("intake-roller", container.m_IntakeSubsystem.getRollerKraken(), "CANivore1");
            recorder.register("turret", container.m_TurretSubsystem.getKraken(), "CANivore1");
            recorder.register("shooter-hood", container.m_ShooterSubsystem.getHoodKraken(), "CANivore1");
            recorder.register("shooter-roller", container.m_ShooterSubsystem.getRollerKraken(), "CANivore1");
            recorder.register("spindexer", container.m_IndexerSubsystem.getRollerKraken(), "CANivore1");
            recorder.register("final-feed", container.m_OuttakeSubsystem.getRollerKraken(), "CANivore1");
            String[] moduleNames = { "front-left", "front-right", "back-left", "back-right" };
            for (int index = 0; index < container.drivetrain.getModules().length; index++) {
                String moduleName = index < moduleNames.length ? moduleNames[index] : Integer.toString(index);
                recorder.register("swerve-" + moduleName + "-drive", container.drivetrain.getModule(index).getDriveMotor(), "CANivore1");
                recorder.register("swerve-" + moduleName + "-steer", container.drivetrain.getModule(index).getSteerMotor(), "CANivore1");
            }
            recorder.writeInventory();
            recorder.startTurretMotionSampling();
        } catch (RuntimeException exception) {
            recorder.healthy = false;
            recorder.healthReason = exception.getClass().getSimpleName() + ": " + exception.getMessage();
            DriverStation.reportWarning("Power logger initialization degraded: " + recorder.healthReason, false);
        }
        return recorder;
    }

    public static synchronized void startDataLogEarly() {
        if (logStartAttempted) return;
        logStartAttempted = true;
        try {
            DataLogManager.start(); // WPILib prefers an attached USB drive and falls back internally.
            DataLogManager.logNetworkTables(false);
            dataLogAvailable = true;
        } catch (RuntimeException exception) {
            dataLogAvailable = false;
        }
    }

    PowerHealthRecorder(
        DoubleSupplier clockSec,
        SampleSink sink,
        PowerSource powerSource,
        Supplier<Context> contextSupplier
    ) {
        this(clockSec, sink, powerSource, contextSupplier, false);
    }

    private PowerHealthRecorder(
        DoubleSupplier clockSec,
        SampleSink sink,
        PowerSource powerSource,
        Supplier<Context> contextSupplier,
        boolean platformHealthEnabled
    ) {
        this.clockSec = clockSec;
        this.sink = sink;
        this.powerSource = powerSource;
        this.contextSupplier = contextSupplier;
        this.platformHealthEnabled = platformHealthEnabled;
    }

    private void register(String name, TalonFX motor, String bus) {
        MotorSignals signals = new MotorSignals(name, bus, motor);
        signals.configureUpdateFrequencies(
            Constants.PowerLogging.MOTOR_ELECTRICAL_RATE_HZ,
            name.equals("turret") ? Constants.PowerLogging.TURRET_MOTION_RATE_HZ : Double.NaN
        );
        motors.add(signals);
        if (name.equals("turret")) turretSignals = signals;
        motor.optimizeBusUtilization();
    }

    private void startTurretMotionSampling() {
        if (!RobotBase.isReal() || turretSignals == null || !sink.available()) return;
        turretMotionNotifier = new Notifier(() -> {
            try {
                sink.number("Power/TurretMotion/TimestampSec", clockSec.getAsDouble());
                turretSignals.recordMotion(sink);
            } catch (RuntimeException exception) {
                healthy = false;
                healthReason = exception.getClass().getSimpleName() + ": " + exception.getMessage();
            }
        });
        turretMotionNotifier.setName("TurretMotionLogger100Hz");
        turretMotionNotifier.startPeriodic(1.0 / Constants.PowerLogging.TURRET_MOTION_RATE_HZ);
    }

    public void periodic() {
        double nowSec = clockSec.getAsDouble();
        try {
            recordFast(nowSec);
            if (nowSec + 1e-9 >= nextSlowSampleSec) {
                recordSlow(nowSec);
                nextSlowSampleSec = nowSec + 1.0 / Constants.PowerLogging.SLOW_HEALTH_RATE_HZ;
            }
            if (nowSec + 1e-9 >= nextDashboardSec) {
                publishDashboard(nowSec);
                nextDashboardSec = nowSec + 1.0 / Constants.PowerLogging.DASHBOARD_RATE_HZ;
            }
        } catch (RuntimeException exception) {
            healthy = false;
            healthReason = exception.getClass().getSimpleName() + ": " + exception.getMessage();
            // Logging is observational and must never inhibit robot control.
        }
    }

    boolean isHealthy() { return healthy && sink.available(); }
    String healthReason() { return healthReason; }

    private void recordFast(double nowSec) {
        sink.number("Power/TimestampSec", nowSec);
        Context context = contextSupplier.get();
        sink.bool("Power/Enabled", context.enabled());
        sink.text("Power/Mode", context.mode());
        sink.bool("Power/ShootRequested", context.shootRequested());
        sink.bool("Power/FeedPermitted", context.feedPermitted());
        sink.bool("Power/ManualOverride", context.manualOverride());
        sink.number("Power/FeedTransitionTimestampSec", context.feedTransitionTimestampSec());
        sink.number("Power/MatchTimeSec", context.matchTimeSec());
        sink.number("Power/ShooterTargetRps", context.shooterTargetRps());
        sink.text("Power/ShotTargetMode", context.shotTargetMode());
        sink.text("Power/ShotModel", context.shotModel());
        sink.text("Power/ShotReason", context.shotReason());
        sink.number("Power/DriveVxMps", context.driveVxMps());
        sink.number("Power/DriveVyMps", context.driveVyMps());
        sink.number("Power/DriveOmegaRadPerSec", context.driveOmegaRadPerSec());
        sink.number("Power/AcceptedVisionAgeSec", context.acceptedVisionAgeSec());
        sink.bool("Power/IntakeExtensionEnabled", context.intakeExtensionEnabled());
        sink.bool("Power/IntakeRollerEnabled", context.intakeRollerEnabled());
        sink.text("Power/IntakeHomingStatus", context.intakeHomingStatus());

        if (powerSource.available()) {
            sink.bool("Power/PDHAvailable", true);
            sink.number("Power/PDHVoltageV", powerSource.voltage());
            sink.number("Power/PDHTotalCurrentA", powerSource.totalCurrent());
            double[] currents = powerSource.channelCurrents();
            for (int channel = 0; channel < currents.length; channel++) {
                sink.number(pdhChannelCurrentKey(channel), currents[channel]);
            }
        } else {
            sink.bool("Power/PDHAvailable", false);
        }
        if (platformHealthEnabled) {
            sink.number("Power/RoboRIOInputVoltageV", RobotController.getInputVoltage());
            sink.number("Power/RoboRIOInputCurrentA", RobotController.getInputCurrent());
            sink.bool("Power/RoboRIOBrownedOut", RobotController.isBrownedOut());
        }

        for (MotorSignals motor : motors) motor.recordFast(sink);
    }

    private void recordSlow(double nowSec) {
        if (platformHealthEnabled) {
            sink.number("Power/RoboRIOCpuTempC", RobotController.getCPUTemp());
            var rioCan = RobotController.getCANStatus();
            sink.number("Power/RoboRIOCAN/Utilization", rioCan.percentBusUtilization);
            sink.number("Power/RoboRIOCAN/BusOffCount", rioCan.busOffCount);
            var canivoreStatus = canivore.getStatus();
            sink.bool("Power/CANivore/Valid", canivoreStatus.Status.isOK());
            sink.number("Power/CANivore/Utilization", canivoreStatus.BusUtilization);
            sink.number("Power/CANivore/BusOffCount", canivoreStatus.BusOffCount);
        }
        sink.text("Power/ConfigurationId", Constants.PowerLogging.CONFIGURATION_ID);
        sink.text("Power/BatteryLabel", Constants.PowerLogging.BATTERY_LABEL);
        for (MotorSignals motor : motors) motor.recordSlow(sink);
        for (ConfigurationHealth.Result result : ConfigurationHealth.snapshot().values()) {
            sink.bool("Power/Config/" + result.deviceName() + "/Applied", result.applied());
            sink.text("Power/Config/" + result.deviceName() + "/Status", result.status());
        }
    }

    private void publishDashboard(double nowSec) {
        SmartDashboard.putBoolean("PowerLogger/Healthy", healthy && sink.available());
        SmartDashboard.putString("PowerLogger/Status", sink.available() ? healthReason : "storage-unavailable");
        SmartDashboard.putBoolean("PowerLogger/PDHAvailable", powerSource.available());
        SmartDashboard.putNumber("PowerLogger/LastWriteSec", nowSec);
        if (powerSource.available()) {
            SmartDashboard.putNumber("PowerLogger/PDHVoltageV", powerSource.voltage());
            SmartDashboard.putNumber("PowerLogger/PDHTotalCurrentA", powerSource.totalCurrent());
        }
    }

    private void writeInventory() {
        sink.text("Power/Inventory/PDH", "REV PDH CAN " + Constants.PowerLogging.REV_PDH_CAN_ID
            + "; channel labels intentionally remain numeric until wiring is verified");
        for (MotorSignals motor : motors) {
            sink.text(
                "Power/Inventory/Motor/" + motor.name(),
                "bus=" + motor.bus() + ", electrical=50Hz, position/velocity=control-owned"
                    + (motor.name().equals("turret") ? " (turret explicitly 100Hz)" : "")
                    + ", temperature/fault/reset=10Hz"
            );
            double[] requestedLimits = requestedCurrentLimits(motor.name());
            if (requestedLimits != null) {
                sink.number("Power/RequestedLimits/" + motor.name() + "/SupplyA", requestedLimits[0]);
                sink.number("Power/RequestedLimits/" + motor.name() + "/StatorA", requestedLimits[1]);
                sink.text("Power/RequestedLimits/" + motor.name() + "/Status",
                    "requested-only; consult Config/<device>/Applied and hardware readback separately");
            }
        }
    }

    static String pdhChannelCurrentKey(int channel) {
        if (channel < 0) throw new IllegalArgumentException("PDH channel must be nonnegative");
        return "Power/PDHChannel/" + channel + "/CurrentA";
    }

    static boolean isFreshSample(boolean statusOk, boolean timestampValid, double ageSec, double maxAgeSec) {
        return statusOk
            && timestampValid
            && Double.isFinite(ageSec)
            && ageSec >= 0.0
            && ageSec <= maxAgeSec;
    }

    private static double[] requestedCurrentLimits(String name) {
        if (name.startsWith("swerve-") && name.endsWith("-drive")) return new double[] {
            TunerConstants.DRIVE_SUPPLY_CURRENT_LIMIT_A, TunerConstants.DRIVE_STATOR_CURRENT_LIMIT_A
        };
        if (name.startsWith("swerve-") && name.endsWith("-steer")) return new double[] {
            TunerConstants.STEER_SUPPLY_CURRENT_LIMIT_A, TunerConstants.STEER_STATOR_CURRENT_LIMIT_A
        };
        return switch (name) {
            case "intake-extension-master", "intake-extension-follower" -> new double[] {
                Constants.Intake.EXTENSION_SUPPLY_CURRENT_LIMIT_A,
                Constants.Intake.EXTENSION_STATOR_CURRENT_LIMIT_A
            };
            case "intake-roller" -> new double[] {
                Constants.Intake.ROLLER_SUPPLY_CURRENT_LIMIT_A,
                Constants.Intake.ROLLER_STATOR_CURRENT_LIMIT_A
            };
            case "turret" -> new double[] {
                Constants.Turret.SUPPLY_CURRENT_LIMIT_A, Constants.Turret.STATOR_CURRENT_LIMIT_A
            };
            case "shooter-hood" -> new double[] {
                Constants.Shooter.HOOD_SUPPLY_CURRENT_LIMIT_A, Constants.Shooter.HOOD_STATOR_CURRENT_LIMIT_A
            };
            case "shooter-roller" -> new double[] {
                Constants.Shooter.ROLLER_SUPPLY_CURRENT_LIMIT_A, Constants.Shooter.ROLLER_STATOR_CURRENT_LIMIT_A
            };
            case "spindexer" -> new double[] {
                Constants.Indexer.ROLLER_SUPPLY_CURRENT_LIMIT_A, Constants.Indexer.ROLLER_STATOR_CURRENT_LIMIT_A
            };
            case "final-feed" -> new double[] {
                Constants.Outtake.ROLLER_SUPPLY_CURRENT_LIMIT_A, Constants.Outtake.ROLLER_STATOR_CURRENT_LIMIT_A
            };
            default -> null;
        };
    }

    public record Context(
        boolean enabled,
        String mode,
        double matchTimeSec,
        boolean shootRequested,
        boolean feedPermitted,
        boolean manualOverride,
        double feedTransitionTimestampSec,
        double shooterTargetRps,
        String shotTargetMode,
        String shotModel,
        String shotReason,
        double driveVxMps,
        double driveVyMps,
        double driveOmegaRadPerSec,
        double acceptedVisionAgeSec,
        boolean intakeExtensionEnabled,
        boolean intakeRollerEnabled,
        String intakeHomingStatus
    ) {
        public Context(
            boolean enabled,
            String mode,
            boolean shootRequested,
            boolean feedPermitted,
            boolean manualOverride,
            double shooterTargetRps
        ) {
            this(enabled, mode, Double.NaN, shootRequested, feedPermitted, manualOverride, Double.NaN,
                shooterTargetRps, "UNKNOWN", "UNKNOWN", "UNKNOWN", Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, false, false, "UNKNOWN");
        }
    }

    interface SampleSink {
        void number(String name, double value);
        void bool(String name, boolean value);
        void text(String name, String value);
        boolean available();

        static SampleSink disabled() {
            return new SampleSink() {
                public void number(String name, double value) {}
                public void bool(String name, boolean value) {}
                public void text(String name, String value) {}
                public boolean available() { return false; }
            };
        }
    }

    interface PowerSource {
        boolean available();
        double voltage();
        double totalCurrent();
        double[] channelCurrents();

        static PowerSource unavailable() {
            return new PowerSource() {
                public boolean available() { return false; }
                public double voltage() { return Double.NaN; }
                public double totalCurrent() { return Double.NaN; }
                public double[] channelCurrents() { return new double[0]; }
            };
        }

        static PowerSource from(PowerDistribution pdh) {
            return new PowerSource() {
                public boolean available() { return true; }
                public double voltage() { return pdh.getVoltage(); }
                public double totalCurrent() { return pdh.getTotalCurrent(); }
                public double[] channelCurrents() { return pdh.getAllCurrents(); }
            };
        }
    }

    private static final class WpiDataLogSink implements SampleSink {
        private final Map<String, DoubleLogEntry> numbers = new LinkedHashMap<>();
        private final Map<String, BooleanLogEntry> booleans = new LinkedHashMap<>();
        private final Map<String, StringLogEntry> strings = new LinkedHashMap<>();

        public synchronized void number(String name, double value) {
            numbers.computeIfAbsent(name, key -> new DoubleLogEntry(DataLogManager.getLog(), key)).append(value);
        }
        public synchronized void bool(String name, boolean value) {
            booleans.computeIfAbsent(name, key -> new BooleanLogEntry(DataLogManager.getLog(), key)).append(value);
        }
        public synchronized void text(String name, String value) {
            strings.computeIfAbsent(name, key -> new StringLogEntry(DataLogManager.getLog(), key)).append(value);
        }
        public boolean available() { return true; }
    }

    private record MotorSignals(
        String name,
        String bus,
        StatusSignal<?> supplyCurrent,
        StatusSignal<?> statorCurrent,
        StatusSignal<?> motorVoltage,
        StatusSignal<?> position,
        StatusSignal<?> velocity,
        StatusSignal<?> temperature,
        StatusSignal<Integer> faults,
        StatusSignal<Boolean> stickyBootDuringEnable,
        StatusSignal<Boolean> stickyBridgeBrownout
    ) {
        MotorSignals(String name, String bus, TalonFX motor) {
            this(
                name,
                bus,
                motor.getSupplyCurrent(),
                motor.getStatorCurrent(),
                motor.getMotorVoltage(),
                motor.getPosition(),
                motor.getVelocity(),
                motor.getDeviceTemp(),
                motor.getFaultField(),
                motor.getStickyFault_BootDuringEnable(),
                motor.getStickyFault_BridgeBrownout()
            );
        }

        void configureUpdateFrequencies(double electricalHertz, double motionHertz) {
            supplyCurrent.setUpdateFrequency(electricalHertz);
            statorCurrent.setUpdateFrequency(electricalHertz);
            motorVoltage.setUpdateFrequency(electricalHertz);
            if (Double.isFinite(motionHertz)) {
                position.setUpdateFrequency(motionHertz);
                velocity.setUpdateFrequency(motionHertz);
            }
            temperature.setUpdateFrequency(Constants.PowerLogging.SLOW_HEALTH_RATE_HZ);
            faults.setUpdateFrequency(Constants.PowerLogging.SLOW_HEALTH_RATE_HZ);
            stickyBootDuringEnable.setUpdateFrequency(Constants.PowerLogging.SLOW_HEALTH_RATE_HZ);
            stickyBridgeBrownout.setUpdateFrequency(Constants.PowerLogging.SLOW_HEALTH_RATE_HZ);
        }

        void recordFast(SampleSink sink) {
            recordSignal(sink, "Power/Motor/" + name + "/SupplyCurrentA", supplyCurrent);
            recordSignal(sink, "Power/Motor/" + name + "/StatorCurrentA", statorCurrent);
            recordSignal(sink, "Power/Motor/" + name + "/MotorVoltageV", motorVoltage);
            recordSignal(sink, "Power/Motor/" + name + "/PositionRot", position);
            recordSignal(sink, "Power/Motor/" + name + "/VelocityRps", velocity);
        }

        void recordSlow(SampleSink sink) {
            recordSignal(sink, "Power/Motor/" + name + "/TemperatureC", temperature);
            sink.number("Power/Motor/" + name + "/FaultField", faults.getValueAsDouble());
            sink.bool("Power/Motor/" + name + "/FaultValid", faults.getStatus().isOK());
            recordBooleanSignal(sink, "Power/Motor/" + name + "/ResetDuringEnable", stickyBootDuringEnable);
            recordBooleanSignal(sink, "Power/Motor/" + name + "/BridgeBrownout", stickyBridgeBrownout);
        }

        void recordMotion(SampleSink sink) {
            recordSignal(sink, "Power/TurretMotion/PositionRot", position);
            recordSignal(sink, "Power/TurretMotion/VelocityRps", velocity);
        }

        private static void recordSignal(SampleSink sink, String key, StatusSignal<?> signal) {
            sink.number(key, signal.getValueAsDouble());
            sink.bool(key + "/Valid", signal.getStatus().isOK() && signal.getTimestamp().isValid());
            sink.number(key + "/AgeSec", signal.getTimestamp().getLatency());
        }

        private static void recordBooleanSignal(SampleSink sink, String key, StatusSignal<Boolean> signal) {
            sink.bool(key, Boolean.TRUE.equals(signal.getValue()));
            sink.bool(key + "/Valid", signal.getStatus().isOK() && signal.getTimestamp().isValid());
            sink.number(key + "/AgeSec", signal.getTimestamp().getLatency());
        }
    }
}

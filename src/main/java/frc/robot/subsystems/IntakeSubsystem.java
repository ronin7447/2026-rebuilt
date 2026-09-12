package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ConfigurationHealth;
import static edu.wpi.first.units.Units.*;

public class IntakeSubsystem extends SubsystemBase {

    private final TalonFX rollerKraken = new TalonFX(Constants.Intake.ROLLER_KRAKEN_ID, "CANivore1");
    private final TalonFX extensionKrakenMaster = new TalonFX(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, "CANivore1");  
    private final TalonFX extensionKrakenFollower = new TalonFX(Constants.Intake.EXTENSION_KRAKEN_FOLLOWER_ID, "CANivore1");

    private final Timer stallTimer = new Timer();

    private double targetPosRot = getIntakePosition();

    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final MotionMagicVelocityVoltage rollerVel = new MotionMagicVelocityVoltage(0.0);

    private boolean mmPosEnabled = false;
    private boolean rollerEnabled = false;
    private int currentLevel = 0;
    private ExtensionProfile activeExtensionProfile = ExtensionProfile.EXTEND;
    private HomingStatus homingStatus = HomingStatus.NOT_RUN;
    private final boolean extensionMasterConfigured;
    private final boolean extensionFollowerConfigured;
    private final boolean rollerConfigured;
    
    public IntakeSubsystem() {

        TalonFXConfiguration cfgFollower = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(Constants.Intake.EXTENSION_STATOR_CURRENT_LIMIT_A))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(Constants.Intake.EXTENSION_SUPPLY_CURRENT_LIMIT_A))
                .withSupplyCurrentLimitEnable(true));

        cfgFollower.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake for default?
        // set slot 0 gains
        var slot0ConfigsFollower = cfgFollower.Slot0;
        slot0ConfigsFollower.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0ConfigsFollower.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0ConfigsFollower.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0ConfigsFollower.kP = 25.0; // An error of 1 rps results in 0.11 V output
        slot0ConfigsFollower.kI = 0; // no output for integrated error
        slot0ConfigsFollower.kD = 0.3; // no output for error derivative'
        
        // set Motion Magic Velocity settings
        var motionMagicConfigsFollower = cfgFollower.MotionMagic;
        motionMagicConfigsFollower.MotionMagicCruiseVelocity = Constants.Intake.EXTENSION_EXTEND_VELOCITY;
        motionMagicConfigsFollower.MotionMagicAcceleration = Constants.Intake.EXTENSION_EXTEND_ACCELERATION;
        motionMagicConfigsFollower.MotionMagicJerk = Constants.Intake.EXTENSION_EXTEND_JERK;

        extensionMasterConfigured = ConfigurationHealth.apply(
            "intake-extension-master-" + Constants.Intake.EXTENSION_KRAKEN_MASTER_ID,
            () -> extensionKrakenMaster.getConfigurator().apply(cfgFollower)
        );
        extensionFollowerConfigured = ConfigurationHealth.apply(
            "intake-extension-follower-" + Constants.Intake.EXTENSION_KRAKEN_FOLLOWER_ID,
            () -> extensionKrakenFollower.getConfigurator().apply(cfgFollower)
        );
        
        extensionKrakenMaster.getPosition().setUpdateFrequency(100);
        extensionKrakenMaster.getVelocity().setUpdateFrequency(100);
        extensionKrakenMaster.getMotorVoltage().setUpdateFrequency(Constants.PowerLogging.MOTOR_ELECTRICAL_RATE_HZ);

        extensionKrakenFollower.getPosition().setUpdateFrequency(100);
        extensionKrakenFollower.getVelocity().setUpdateFrequency(100);
        extensionKrakenFollower.getMotorVoltage().setUpdateFrequency(Constants.PowerLogging.MOTOR_ELECTRICAL_RATE_HZ);

        // in init function
        var rollerTalonFXConfigs = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(Constants.Intake.ROLLER_STATOR_CURRENT_LIMIT_A))
                .withSupplyCurrentLimit(Amps.of(Constants.Intake.ROLLER_SUPPLY_CURRENT_LIMIT_A))
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true));

        // set slot 0 gains
        var slot0Configs = rollerTalonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.22; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = rollerTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 200; // Softer ramp to reduce intake current spikes
        motionMagicConfigs.MotionMagicJerk = 2000;

        stallTimer.stop();
        stallTimer.reset();

        rollerConfigured = ConfigurationHealth.apply(
            "intake-roller-" + Constants.Intake.ROLLER_KRAKEN_ID,
            () -> rollerKraken.getConfigurator().apply(rollerTalonFXConfigs)
        );

        zeroEncoder();
    }

    public TalonFX getExtensionKrakenMaster() {

        return extensionKrakenMaster;
    }

    public TalonFX getExtensionKrakenFollower() {

        return extensionKrakenFollower;
    }

    public TalonFX getRollerKraken() {
        return rollerKraken;
    }

    public void runRoller(double vel) {
        if (rollerConfigured && Double.isFinite(vel)) {
            rollerKraken.setControl(rollerVel.withVelocity(vel));
        } else {
            rollerKraken.stopMotor();
        }
    }

    public double getIntakePosition() {
        return extensionKrakenMaster.getPosition().getValueAsDouble();
    }

    public void enableExtension() {
        mmPosEnabled = extensionMasterConfigured;
    }

    public void disableExtension() {
        mmPosEnabled = false;

        extensionKrakenMaster.setControl(new VoltageOut(0));
    }

    public void enableRoller() {
        rollerEnabled = rollerConfigured;
    }

    public void disableRoller() {
        rollerEnabled = false;
    }

    public void zeroEncoder() {

        mmPosEnabled = false;

        extensionKrakenMaster.setPosition(0);
    }

    public double getTargetRot() {

        return targetPosRot;
    }

    public void commandExtensionPosition(double targetPosRot) {
        setIntakePosition(targetPosRot);
        enableExtension();
    }

    public boolean isExtensionAtTarget() {
        return Math.abs(getIntakePosition() - getTargetRot()) <= Constants.Intake.INTAKE_TARGET_ROT_TOLERANCE;
    }

    public void setIntakePosition(double targetPosRot) {
        updateExtensionProfile(targetPosRot);
        this.targetPosRot = targetPosRot;
    }

    private void updateExtensionProfile(double newTargetPosRot) {
        ExtensionProfile desiredProfile = newTargetPosRot < getIntakePosition()
            ? ExtensionProfile.COLLAPSE
            : ExtensionProfile.EXTEND;

        if (desiredProfile == activeExtensionProfile) {
            return;
        }

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        if (desiredProfile == ExtensionProfile.COLLAPSE) {
            motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.EXTENSION_COLLAPSE_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.EXTENSION_COLLAPSE_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk = Constants.Intake.EXTENSION_COLLAPSE_JERK;
        } else {
            motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.EXTENSION_EXTEND_VELOCITY;
            motionMagicConfigs.MotionMagicAcceleration = Constants.Intake.EXTENSION_EXTEND_ACCELERATION;
            motionMagicConfigs.MotionMagicJerk = Constants.Intake.EXTENSION_EXTEND_JERK;
        }

        if (ConfigurationHealth.apply(
            "intake-extension-profile-" + Constants.Intake.EXTENSION_KRAKEN_MASTER_ID,
            () -> extensionKrakenMaster.getConfigurator().apply(motionMagicConfigs)
        )) {
            activeExtensionProfile = desiredProfile;
        }
    }

    public int getCurrentLevel() {
        return currentLevel;
    }
    
    public boolean isStallDetected() {
        StatusSignal<Current> statorCurrentSig = extensionKrakenMaster.getStatorCurrent();
        StatusSignal<AngularVelocity> velocitySig = extensionKrakenMaster.getVelocity();

        statorCurrentSig.refresh();
        velocitySig.refresh();

        double amps = statorCurrentSig.getValueAsDouble();
        double velocityRps = Math.abs(velocitySig.getValueAsDouble());
        boolean stalledNow = amps >= Constants.Intake.INTAKE_STALL_CURRENT
            && velocityRps <= Constants.Intake.INTAKE_STALL_VELOCITY;

        if (stalledNow) {
            if (!stallTimer.isRunning()) {
                stallTimer.reset();
                stallTimer.start();
            }
        } else {
            stallTimer.stop();
            stallTimer.reset();
        }

        return stallTimer.hasElapsed(Constants.Intake.STALL_TIME_S);
    }

    public void resetStallDetection() {
        stallTimer.stop();
        stallTimer.reset();
    }

    public boolean beginHoming() {
        disableExtension();
        disableRoller();
        resetStallDetection();
        homingStatus = HomingStatus.RUNNING;
        if (extensionMasterConfigured && extensionFollowerConfigured) {
            extensionKrakenMaster.setControl(new VoltageOut(Constants.Intake.HOMING_VOLTAGE));
            return true;
        } else {
            homingStatus = HomingStatus.INTERRUPTED;
            return false;
        }
    }

    public void finishHoming(HomingStatus status) {
        extensionKrakenMaster.setControl(new VoltageOut(0.0));
        homingStatus = status;
        if (status == HomingStatus.SUCCESS) {
            zeroEncoder();
        }
    }

    public HomingStatus getHomingStatus() {
        return homingStatus;
    }

    public boolean isExtensionEnabled() {
        return mmPosEnabled;
    }

    public boolean isRollerEnabled() {
        return rollerEnabled;
    }

    @Override
    public void periodic() {

        if (mmPosEnabled && extensionMasterConfigured) {

            extensionKrakenMaster.setControl(mmPos.withPosition(targetPosRot));
        }

        if (extensionFollowerConfigured) {
            extensionKrakenFollower.setControl(new Follower(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, MotorAlignmentValue.Opposed));
        }
        // extensionKrakenFollower.setControl(new Follower(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, MotorAlignmentValue.Aligned));
        
        if (rollerEnabled && rollerConfigured) {

            rollerKraken.setControl(rollerVel.withVelocity(Constants.Intake.ROLLER_VELOCITY));
        } else {

            rollerKraken.setControl(rollerVel.withVelocity(0));     
        }

        SmartDashboard.putNumber("StatorCurrent-ExtensionMaster", getExtensionKrakenMaster().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("StatorCurrent-ExtensionFollower", getExtensionKrakenFollower().getStatorCurrent().getValueAsDouble());
        SmartDashboard.putString("Intake/HomingStatus", homingStatus.name());


        
    }

    private enum ExtensionProfile {
        EXTEND,
        COLLAPSE
    }

    public enum HomingStatus {
        NOT_RUN,
        RUNNING,
        SUCCESS,
        TIMEOUT,
        INTERRUPTED
    }
}

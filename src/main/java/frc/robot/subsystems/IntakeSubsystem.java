package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
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
    
    public IntakeSubsystem() {

        TalonFXConfiguration cfgFollower = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(100))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(100))
                .withSupplyCurrentLimitEnable(true));

        cfgFollower.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake for default?
        // set slot 0 gains
        var slot0ConfigsFollower = cfgFollower.Slot0;
        slot0ConfigsFollower.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0ConfigsFollower.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0ConfigsFollower.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0ConfigsFollower.kP = 45.0; // An error of 1 rps results in 0.11 V output
        slot0ConfigsFollower.kI = 0; // no output for integrated error
        slot0ConfigsFollower.kD = 0.3; // no output for error derivative'
        
        // set Motion Magic Velocity settings
        var motionMagicConfigsFollower = cfgFollower.MotionMagic;
        motionMagicConfigsFollower.MotionMagicCruiseVelocity = Constants.Intake.EXTENSION_VELOCITY;
        motionMagicConfigsFollower.MotionMagicAcceleration = Constants.Intake.EXTENSION_ACCELERATION; // Target acceleration of 40 rps/s 
        motionMagicConfigsFollower.MotionMagicJerk = Constants.Intake.EXTENSION_JERK; // Target jerk of 400 rps/s/s 

        extensionKrakenMaster.getConfigurator().apply(cfgFollower);
        extensionKrakenFollower.getConfigurator().apply(cfgFollower);
        
        extensionKrakenMaster.getPosition().setUpdateFrequency(100);
        extensionKrakenMaster.getVelocity().setUpdateFrequency(100);
        extensionKrakenMaster.getMotorVoltage().setUpdateFrequency(1000);
        extensionKrakenMaster.optimizeBusUtilization();

        extensionKrakenFollower.getPosition().setUpdateFrequency(100);
        extensionKrakenFollower.getVelocity().setUpdateFrequency(100);
        extensionKrakenFollower.getMotorVoltage().setUpdateFrequency(1000);
        extensionKrakenFollower.optimizeBusUtilization();

        extensionKrakenMaster.setNeutralMode(NeutralModeValue.Brake);
        extensionKrakenFollower.setNeutralMode(NeutralModeValue.Brake);

        // in init function
        var rollerTalonFXConfigs = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(80))
                .withSupplyCurrentLimit(40)
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
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        stallTimer.stop();
        stallTimer.reset();

        rollerKraken.getConfigurator().apply(rollerTalonFXConfigs);

        zeroEncoder();
    }

    public TalonFX getExtensionKrakenMaster() {

        return extensionKrakenMaster;
    }

    public TalonFX getExtensionKrakenFollower() {

        return extensionKrakenFollower;
    }

    public void runRoller(double vel) {
        rollerKraken.setControl(rollerVel.withVelocity(vel));
    }

    public double getIntakePosition() {
        return extensionKrakenMaster.getPosition().getValueAsDouble();
    }

    public void enableExtension() {
        mmPosEnabled = true;
    }

    public void disableExtension() {
        mmPosEnabled = false;

        extensionKrakenMaster.setControl(new VoltageOut(0));
    }

    public void enableRoller() {
        rollerEnabled = true;
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
        this.targetPosRot = targetPosRot;
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

    @Override
    public void periodic() {

        if (mmPosEnabled) {

            extensionKrakenMaster.setControl(mmPos.withPosition(targetPosRot));
        }

        extensionKrakenFollower.setControl(new Follower(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, MotorAlignmentValue.Opposed));
        // extensionKrakenFollower.setControl(new Follower(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, MotorAlignmentValue.Aligned));
        
        if (rollerEnabled) {

            rollerKraken.setControl(rollerVel.withVelocity(Constants.Intake.ROLLER_VELOCITY));
        } else {

            rollerKraken.setControl(rollerVel.withVelocity(0));     
        }

        
    }
}

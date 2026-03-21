package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX rollerKraken = new TalonFX(Constants.Intake.ROLLER_KRAKEN_ID);
    private final TalonFX extensionKrakenMaster = new TalonFX(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID);  
    private final TalonFX extensionKrakenFollower = new TalonFX(Constants.Intake.EXTENSION_KRAKEN_FOLLOWER_ID);

    private double targetPosRot = getIntakePosition();

    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final MotionMagicVelocityVoltage rollerVel = new MotionMagicVelocityVoltage(0.0);

    private boolean mmPosEnabled = false;
    private boolean rollerEnabled = false;
    private int currentLevel = 0;
    
    public IntakeSubsystem() {


        TalonFXConfiguration cfgFollower = new TalonFXConfiguration();

        cfgFollower.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake for default?
        // set slot 0 gains
        var slot0ConfigsFollower = cfgFollower.Slot0;
        slot0ConfigsFollower.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0ConfigsFollower.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0ConfigsFollower.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0ConfigsFollower.kP = 35.0; // An error of 1 rps results in 0.11 V output
        slot0ConfigsFollower.kI = 0; // no output for integrated error
        slot0ConfigsFollower.kD = 0.2; // no output for error derivative'
        
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


        // in init function
        var rollerTalonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = rollerTalonFXConfigs.Slot0;
        slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 0.11; // An error of 1 rps results in 0.11 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0; // no output for error derivative

        // set Motion Magic Velocity settings
        var motionMagicConfigs = rollerTalonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)

        rollerKraken.getConfigurator().apply(rollerTalonFXConfigs);

    }

    public TalonFX getExtensionKrakenMaster(){
        return extensionKrakenMaster;
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
    }

    public void enableRoller() {
        rollerEnabled = true;
    }

    public void disableRoller() {
        rollerEnabled = false;
    }

    // public void setMotorCoast() {
    //     extensionKrakenMaster.set
    // }

    public void setIntakePosition(double targetPosRot) {
        this.targetPosRot = targetPosRot;
    }

    public int getCurrentLevel() {
        return currentLevel;
    }
    
    @Override
    public void periodic() {

        if (mmPosEnabled) {
            extensionKrakenMaster.setControl(mmPos.withPosition(targetPosRot));
            extensionKrakenFollower.setControl(new Follower(Constants.Intake.EXTENSION_KRAKEN_MASTER_ID, MotorAlignmentValue.Opposed));
            if (Math.abs(this.getIntakePosition() - targetPosRot) <= Constants.Intake.INTAKE_TARGET_ROT_TOLERANCE) {
                this.disableExtension();
                if (Math.abs(this.getIntakePosition() - Constants.Intake.INTAKE_COLLAPSED_POS) <= Constants.Intake.INTAKE_TARGET_ROT_TOLERANCE) {
                    // when we collapse our intake
                    // we stop our motor
                    extensionKrakenMaster.stopMotor();
                    extensionKrakenFollower.stopMotor();
                }
                // otherwise pid will hold it
            }
        }
        
        if (rollerEnabled) {
            rollerKraken.setControl(rollerVel.withVelocity(Constants.Intake.ROLLER_VELOCITY));
        }

        
    }
}
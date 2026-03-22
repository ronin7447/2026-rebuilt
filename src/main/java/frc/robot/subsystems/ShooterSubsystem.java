package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX hoodKraken = new TalonFX(Constants.Shooter.HOOD_KRAKEN_ID);
    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private double targetPosRot = getHoodPosition(); 


    public ShooterSubsystem() {

        // reset position when init subsystem
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake for default?
        // set slot 0 gains
        var slot0Configs = cfg.Slot0;
        slot0Configs.kS = Constants.Shooter.KS;
        slot0Configs.kV = Constants.Shooter.KV;
        slot0Configs.kA = Constants.Shooter.KA;
        slot0Configs.kP = Constants.Shooter.KP;
        slot0Configs.kI = Constants.Shooter.KI;
        slot0Configs.kD = Constants.Shooter.KD;
        
        // set Motion Magic Velocity settings
        var motionMagicConfigs = cfg.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Turret.CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Turret.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Turret.JERK;

        hoodKraken.getConfigurator().apply(cfg);
        hoodKraken.getPosition().setUpdateFrequency(100);
        hoodKraken.getVelocity().setUpdateFrequency(100);
        hoodKraken.optimizeBusUtilization();
    }
    
     public TalonFX getHoodKraken(){
        return hoodKraken;
    }


    public void setTargetPositionRotation(double pos) {

        targetPosRot = pos;
    }


    public double getHoodPosition() {
        return hoodKraken.getPosition().getValueAsDouble();
    }

    @Override
    public void periodic() {

        hoodKraken.setControl(mmPos.withPosition(targetPosRot));
    }
}

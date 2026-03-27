package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX hoodKraken = new TalonFX(Constants.Shooter.HOOD_KRAKEN_ID, "CANivore1");
    private final TalonFX rollerKraken;
    
    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final MotionMagicVelocityVoltage rollerVel = new MotionMagicVelocityVoltage(0.0);
    private double targetPosRot = getHoodPosition(); 
    private double targetRollerVelRps = Constants.Shooter.ROLLER_VELOCITY;
    private boolean rollerEnabled = false;


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
        // hoodKraken.getPosition().setUpdateFrequency(100);
        // hoodKraken.getVelocity().setUpdateFrequency(100);
        // hoodKraken.optimizeBusUtilization();

        rollerKraken = new TalonFX(Constants.Shooter.ROLLER_KRAKEN_ID, "CANivore1");

        TalonFXConfiguration rollerCfg = new TalonFXConfiguration();

        var rollerSlot0Configs = rollerCfg.Slot0;
        rollerSlot0Configs.kS = Constants.Shooter.ROLLER_KS;
        rollerSlot0Configs.kV = Constants.Shooter.ROLLER_KV;
        rollerSlot0Configs.kA = Constants.Shooter.ROLLER_KA;
        rollerSlot0Configs.kP = Constants.Shooter.ROLLER_KP;
        rollerSlot0Configs.kI = Constants.Shooter.ROLLER_KI;
        rollerSlot0Configs.kD = Constants.Shooter.ROLLER_KD;

        var rollerMotionMagicConfigs = rollerCfg.MotionMagic;
        rollerMotionMagicConfigs.MotionMagicAcceleration = Constants.Shooter.ROLLER_ACCELERATION;
        rollerMotionMagicConfigs.MotionMagicJerk = Constants.Shooter.ROLLER_JERK;

        rollerKraken.getConfigurator().apply(rollerCfg);
        // rollerKraken.getVelocity().setUpdateFrequency(100);
        // rollerKraken.getMotorVoltage().setUpdateFrequency(1000);
        // rollerKraken.optimizeBusUtilization();
    }
    
     public TalonFX getHoodKraken(){
        return hoodKraken;
    }

    public TalonFX getRollerKraken() {
        return rollerKraken;
    }


    public void setTargetPositionRotation(double pos) {

        targetPosRot = pos;
    }

    public void setRollerTargetVelocity(double velRps) {
        targetRollerVelRps = velRps;
    }

    public void runRoller(double velRps) {
        setRollerTargetVelocity(velRps);
        enableRoller();
    }

    public void enableRoller() {
        rollerEnabled = rollerKraken != null;
    }

    public void disableRoller() {
        rollerEnabled = false;
        if (rollerKraken != null) {
            rollerKraken.stopMotor();
        }
    }


    public double getHoodPosition() {
        return hoodKraken.getPosition().getValueAsDouble();
    }

    public double getRollerVelocity() {
        if (rollerKraken == null) {
            return 0.0;
        }
        return rollerKraken.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {

        // hoodKraken.setControl(mmPos.withPosition(targetPosRot));

        if (rollerEnabled && rollerKraken != null) {
            rollerKraken.setControl(rollerVel.withVelocity(targetRollerVelRps));
        }
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

import static edu.wpi.first.units.Units.*;

public class IndexerSubsystem extends SubsystemBase {
    private final TalonFX rollerKraken;
    
    private final MotionMagicVelocityVoltage rollerVel = new MotionMagicVelocityVoltage(0.0);
    private double targetRollerVelRps = Constants.Outtake.ROLLER_VELOCITY;
    private boolean rollerEnabled = false;


    public IndexerSubsystem() {


        rollerKraken = new TalonFX(Constants.Indexer.ROLLER_KRAKEN_ID, "CANivore1");

        TalonFXConfiguration rollerCfg = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(Constants.Indexer.STATOR_CURRENT_LIMIT_A))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(Constants.Indexer.SUPPLY_CURRENT_LIMIT_A))
                .withSupplyCurrentLimitEnable(true));

        var rollerSlot0Configs = rollerCfg.Slot0;
        rollerSlot0Configs.kS = Constants.Indexer.ROLLER_KS;
        rollerSlot0Configs.kV = Constants.Indexer.ROLLER_KV;
        rollerSlot0Configs.kA = Constants.Indexer.ROLLER_KA;
        rollerSlot0Configs.kP = Constants.Indexer.ROLLER_KP;
        rollerSlot0Configs.kI = Constants.Indexer.ROLLER_KI;
        rollerSlot0Configs.kD = Constants.Indexer.ROLLER_KD;

        var rollerMotionMagicConfigs = rollerCfg.MotionMagic;
        rollerMotionMagicConfigs.MotionMagicAcceleration = Constants.Indexer.ROLLER_ACCELERATION;
        rollerMotionMagicConfigs.MotionMagicJerk = Constants.Indexer.ROLLER_JERK;

        rollerKraken.getConfigurator().apply(rollerCfg);
        // rollerKraken.getVelocity().setUpdateFrequency(100);
        // rollerKraken.getMotorVoltage().setUpdateFrequency(1000);
        // rollerKraken.optimizeBusUtilization();
    }
    

    public TalonFX getRollerKraken() {
        return rollerKraken;
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

    public double getRollerVelocity() {
        if (rollerKraken == null) {
            return 0.0;
        }
        return rollerKraken.getVelocity().getValueAsDouble();
    }

    @Override
    public void periodic() {

        if (rollerEnabled && rollerKraken != null) {
            rollerKraken.setControl(rollerVel.withVelocity(targetRollerVelRps));
        }
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.util.ConfigurationHealth;
import static edu.wpi.first.units.Units.*;

public class OuttakeSubsystem extends SubsystemBase {
    private final TalonFX rollerKraken;
    
    private final MotionMagicVelocityVoltage rollerVel = new MotionMagicVelocityVoltage(0.0);
    private double targetRollerVelRps = Constants.Outtake.ROLLER_VELOCITY;
    private boolean rollerEnabled = false;
    private final boolean configured;


    public OuttakeSubsystem() {


        rollerKraken = new TalonFX(Constants.Outtake.ROLLER_KRAKEN_ID, "CANivore1");

        TalonFXConfiguration rollerCfg = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(Constants.Outtake.ROLLER_STATOR_CURRENT_LIMIT_A))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(Constants.Outtake.ROLLER_SUPPLY_CURRENT_LIMIT_A))
                .withSupplyCurrentLimitEnable(true)
                );

        var rollerSlot0Configs = rollerCfg.Slot0;
        rollerSlot0Configs.kS = Constants.Outtake.ROLLER_KS;
        rollerSlot0Configs.kV = Constants.Outtake.ROLLER_KV;
        rollerSlot0Configs.kA = Constants.Outtake.ROLLER_KA;
        rollerSlot0Configs.kP = Constants.Outtake.ROLLER_KP;
        rollerSlot0Configs.kI = Constants.Outtake.ROLLER_KI;
        rollerSlot0Configs.kD = Constants.Outtake.ROLLER_KD;

        var rollerMotionMagicConfigs = rollerCfg.MotionMagic;
        rollerMotionMagicConfigs.MotionMagicAcceleration = Constants.Outtake.ROLLER_ACCELERATION;
        rollerMotionMagicConfigs.MotionMagicJerk = Constants.Outtake.ROLLER_JERK;

        configured = ConfigurationHealth.apply(
            "final-feed-" + Constants.Outtake.ROLLER_KRAKEN_ID,
            () -> rollerKraken.getConfigurator().apply(rollerCfg)
        );
        // rollerKraken.getVelocity().setUpdateFrequency(100);
        // rollerKraken.optimizeBusUtilization();
    }
    

    public TalonFX getRollerKraken() {
        return rollerKraken;
    }


    public void setRollerTargetVelocity(double velRps) {
        if (!Double.isFinite(velRps)) return;
        targetRollerVelRps = velRps;
    }

    public void runRoller(double velRps) {
        setRollerTargetVelocity(velRps);
        enableRoller();
    }

    public void enableRoller() {
        rollerEnabled = configured && rollerKraken != null;
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

        if (rollerEnabled && configured && rollerKraken != null) {
            rollerKraken.setControl(rollerVel.withVelocity(targetRollerVelRps));
        } else if (!configured) {
            rollerKraken.stopMotor();
        }
    }
}

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;


public class TurretSubsystem extends SubsystemBase{
    
    private final TalonFX turretKraken = new TalonFX(Constants.Turret.TURRET_KRAKEN_ID);

    // motion magic
    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final VoltageOut openLoopVolts = new VoltageOut(0.0);

    // signals for stall detection
    private final StatusSignal<Current> statorCurrentSig = turretKraken.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocitySig = turretKraken.getVelocity();

    private double targetPosRot = getPosition(); 
    private final Timer stallTimer = new Timer(); // a timer for stall check

    // stall check
    private static final double kStallCurrentA = Constants.Turret.STALL_CURRENT_A;
    private static final double kStallVelRps = Constants.Turret.STALL_VEL_RPS;
    private static final double kStallTimeS = Constants.Turret.STALL_TIME_S;

    private static final double kPosToleranceRot = Constants.Turret.POSITION_TOLERANCE_ROT;

    private boolean positionMode = true;
    


    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake; // brake for default?
        // set slot 0 gains
        var slot0Configs = cfg.Slot0;
        slot0Configs.kS = Constants.Turret.KS;
        slot0Configs.kV = Constants.Turret.KV;
        slot0Configs.kA = Constants.Turret.KA;
        slot0Configs.kP = Constants.Turret.KP;
        slot0Configs.kI = Constants.Turret.KI;
        slot0Configs.kD = Constants.Turret.KD;
        
        // set Motion Magic Velocity settings
        var motionMagicConfigs = cfg.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Turret.CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Turret.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Turret.JERK;

        turretKraken.getConfigurator().apply(cfg);
        stallTimer.stop();
        stallTimer.reset();

        turretKraken.getPosition().setUpdateFrequency(100);
        turretKraken.getVelocity().setUpdateFrequency(100);
        turretKraken.getStatorCurrent().setUpdateFrequency(100);
        turretKraken.getMotorVoltage().setUpdateFrequency(1000);
        turretKraken.optimizeBusUtilization();
    }

    public TalonFX getKraken(){
        return turretKraken;
    }

    public void stopMotor(){
        
        turretKraken.stopMotor();
        
    }

    public boolean hitSoftLimHigh(){
        return getPosition() >= (Constants.Turret.SOFT_LIMIT_HIGH - Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    public boolean hitSoftLimLow(){
        return getPosition() <= (Constants.Turret.SOFT_LIMIT_LOW + Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    /**
     * TODO: Stall Check
     * @return
     */

     /**
      * TODO: Reset Position to Zero
      * @return
      */
    public double getPosition() {
        return turretKraken.getPosition().getValueAsDouble();
    }
    public double getVelocityRps() {
        return turretKraken.getVelocity().getValueAsDouble();
    }
    public boolean atTarget() {
        return Math.abs(targetPosRot - getPosition()) <= kPosToleranceRot;
    }
    

    public void setTargetPositionRotation(double pos) {
        positionMode = true;
        targetPosRot = Math.max(Constants.Turret.SOFT_LIMIT_LOW, Math.min(pos, Constants.Turret.SOFT_LIMIT_HIGH));
    }

    public double getTargetPositionRotation() {
        return targetPosRot;
    }

    public void setPositionMode(boolean mode) {
        positionMode = mode;
        if (!positionMode) {
            turretKraken.setControl(openLoopVolts.withOutput(0.0));
        }
    }

    public boolean getPositionMode() {
        return positionMode;
    }

    public void setVoltage(double output) {
        if (!positionMode) {
            turretKraken.setControl(openLoopVolts.withOutput(output));
        }
    }

    public void zeroEncoder() {
        turretKraken.setPosition(0);
        setTargetPositionRotation(0);
    }

    public void zeroEncoderAtHardStop() {
        setVoltage(0.0);
        zeroEncoder();
    }

    public boolean isStallDetected() {
        statorCurrentSig.refresh();
        velocitySig.refresh();

        double amps = statorCurrentSig.getValueAsDouble();
        double vel = Math.abs(velocitySig.getValueAsDouble());

        boolean stalledNow = amps >= kStallCurrentA && vel <= kStallVelRps;

        if (stalledNow) {
            if (!stallTimer.isRunning()) {
                stallTimer.reset();
                stallTimer.start();
            }
        } else {
            stallTimer.stop();
            stallTimer.reset();
        }

        return stallTimer.hasElapsed(kStallTimeS);
    }


    @Override
    public void periodic() {

        if (positionMode) {
            turretKraken.setControl(mmPos.withPosition(targetPosRot));
        }
    }
    
    
}

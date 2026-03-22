package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretKraken = new TalonFX(Constants.Turret.TURRET_KRAKEN_ID);

    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final VoltageOut openLoopVolts = new VoltageOut(0.0);

    private final StatusSignal<Current> statorCurrentSig = turretKraken.getStatorCurrent();
    private final StatusSignal<AngularVelocity> velocitySig = turretKraken.getVelocity();

    private double targetPosRot = 0.0;
    private final Timer stallTimer = new Timer();

    private boolean positionMode = true;
    private boolean autoAimActive = false;
    private boolean poseFreshForShooting = false;
    private boolean inAllianceZone = false;
    private boolean visionHealthy = false;
    private boolean autoAimSuppressed = false;
    private boolean goalClampedBySoftwareLimit = false;

    private Rotation2d desiredTurretAngle = Rotation2d.kZero;
    private Translation2d turretPivotFieldPosition = Translation2d.kZero;
    private Translation2d selectedHubPoint = Translation2d.kZero;
    private String allianceName = "Unknown";

    private BooleanSupplier shooterReadySupplier = () -> true;

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration();

        cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        var slot0Configs = cfg.Slot0;
        slot0Configs.kS = Constants.Turret.KS;
        slot0Configs.kV = Constants.Turret.KV;
        slot0Configs.kA = Constants.Turret.KA;
        slot0Configs.kP = Constants.Turret.KP;
        slot0Configs.kI = Constants.Turret.KI;
        slot0Configs.kD = Constants.Turret.KD;

        var motionMagicConfigs = cfg.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Turret.CRUISE_VELOCITY;
        motionMagicConfigs.MotionMagicAcceleration = Constants.Turret.ACCELERATION;
        motionMagicConfigs.MotionMagicJerk = Constants.Turret.JERK;

        turretKraken.getConfigurator().apply(cfg);
        turretKraken.getPosition().setUpdateFrequency(100);
        turretKraken.getVelocity().setUpdateFrequency(100);
        turretKraken.getStatorCurrent().setUpdateFrequency(100);
        turretKraken.getMotorVoltage().setUpdateFrequency(1000);
        turretKraken.optimizeBusUtilization();

        targetPosRot = getPosition();
        stallTimer.stop();
        stallTimer.reset();
    }

    public TalonFX getKraken() {
        return turretKraken;
    }

    public void stopMotor() {
        turretKraken.stopMotor();
    }

    public boolean hitSoftLimHigh() {
        return getPosition() >= (Constants.Turret.SOFT_LIMIT_HIGH - Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    public boolean hitSoftLimLow() {
        return getPosition() <= (Constants.Turret.SOFT_LIMIT_LOW + Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    public double getPosition() {
        return turretKraken.getPosition().getValueAsDouble();
    }

    public double getVelocityRps() {
        return turretKraken.getVelocity().getValueAsDouble();
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromRotations(getPosition());
    }

    public Rotation2d getTurretGoal() {
        return Rotation2d.fromRotations(targetPosRot);
    }

    public Rotation2d getDesiredTurretAngle() {
        return desiredTurretAngle;
    }

    public Rotation2d getAimError() {
        return getTurretGoal().minus(getTurretAngle());
    }

    public double getTurretAimErrorDeg() {
        return getAimError().getDegrees();
    }

    public boolean atTarget() {
        return atGoal();
    }

    public boolean atGoal() {
        return Math.abs(getTurretAimErrorDeg()) <= Constants.Turret.AIM_TOLERANCE_DEG;
    }

    public boolean isTurretAtAimGoal() {
        return atGoal();
    }

    public void setTargetPositionRotation(double posRot) {
        positionMode = true;
        targetPosRot = Math.max(Constants.Turret.SOFT_LIMIT_LOW, Math.min(posRot, Constants.Turret.SOFT_LIMIT_HIGH));
    }

    public double getTargetPositionRotation() {
        return targetPosRot;
    }

    public void setTurretGoal(Rotation2d goal) {
        Rotation2d clampedGoal = clampToSoftwareLimits(goal);
        desiredTurretAngle = goal;
        goalClampedBySoftwareLimit = Math.abs(clampedGoal.minus(goal).getDegrees()) > 1e-3;
        setTargetPositionRotation(clampedGoal.getRotations());
    }

    public Rotation2d clampToSoftwareLimits(Rotation2d requested) {
        double requestedRot = requested.getRotations();
        double currentRot = getPosition();
        double wrappedCandidate = requestedRot + Math.rint(currentRot - requestedRot);
        double clampedRot = Math.max(Constants.Turret.SOFT_LIMIT_LOW, Math.min(wrappedCandidate, Constants.Turret.SOFT_LIMIT_HIGH));
        return Rotation2d.fromRotations(clampedRot);
    }

    public void holdCurrentPosition() {
        setTurretGoal(getTurretAngle());
    }

    public void park() {
        setTurretGoal(Constants.Turret.PARK_ANGLE);
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
        boolean stalledNow = amps >= Constants.Turret.STALL_CURRENT_A && vel <= Constants.Turret.STALL_VEL_RPS;

        if (stalledNow) {
            if (!stallTimer.isRunning()) {
                stallTimer.reset();
                stallTimer.start();
            }
        } else {
            stallTimer.stop();
            stallTimer.reset();
        }

        return stallTimer.hasElapsed(Constants.Turret.STALL_TIME_S);
    }

    public void setShooterReadySupplier(BooleanSupplier shooterReadySupplier) {
        this.shooterReadySupplier = shooterReadySupplier != null ? shooterReadySupplier : () -> true;
    }

    public void setAutoAimSuppressed(boolean suppressed) {
        autoAimSuppressed = suppressed;
    }

    public boolean isAutoAimSuppressed() {
        return autoAimSuppressed;
    }

    public boolean isAutoAimActive() {
        return autoAimActive;
    }

    public boolean isPoseFreshForShooting() {
        return poseFreshForShooting;
    }

    public boolean isOnSoftwareLimit() {
        return goalClampedBySoftwareLimit || hitSoftLimHigh() || hitSoftLimLow();
    }

    public boolean isShotReady() {
        return autoAimActive
            && poseFreshForShooting
            && isTurretAtAimGoal()
            && !isOnSoftwareLimit()
            && shooterReadySupplier.getAsBoolean();
    }

    public void updateAimState(
        String allianceName,
        boolean inAllianceZone,
        boolean visionHealthy,
        boolean poseFreshForShooting,
        boolean autoAimActive,
        Translation2d turretPivotFieldPosition,
        Translation2d selectedHubPoint,
        Rotation2d desiredTurretAngle
    ) {
        this.allianceName = allianceName;
        this.inAllianceZone = inAllianceZone;
        this.visionHealthy = visionHealthy;
        this.poseFreshForShooting = poseFreshForShooting;
        this.autoAimActive = autoAimActive;
        this.turretPivotFieldPosition = turretPivotFieldPosition;
        this.selectedHubPoint = selectedHubPoint;
        this.desiredTurretAngle = desiredTurretAngle;
    }

    @Override
    public void periodic() {
        if (positionMode) {
            turretKraken.setControl(mmPos.withPosition(targetPosRot));
        }

        publishTelemetry();
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("Turret/ActualAngleDeg", getTurretAngle().getDegrees());
        SmartDashboard.putNumber("Turret/GoalAngleDeg", getTurretGoal().getDegrees());
        SmartDashboard.putNumber("Turret/DesiredAngleDeg", desiredTurretAngle.getDegrees());
        SmartDashboard.putNumber("Turret/AimErrorDeg", getTurretAimErrorDeg());
        SmartDashboard.putBoolean("Turret/AtAimGoal", isTurretAtAimGoal());
        SmartDashboard.putBoolean("Turret/ShotReady", isShotReady());
        SmartDashboard.putBoolean("Turret/AutoAimActive", autoAimActive);
        SmartDashboard.putBoolean("Turret/AutoAimSuppressed", autoAimSuppressed);
        SmartDashboard.putBoolean("Turret/InAllianceZone", inAllianceZone);
        SmartDashboard.putBoolean("Turret/VisionHealthy", visionHealthy);
        SmartDashboard.putBoolean("Turret/PoseFreshForShot", poseFreshForShooting);
        SmartDashboard.putBoolean("Turret/OnSoftwareLimit", isOnSoftwareLimit());
        SmartDashboard.putString("Turret/Alliance", allianceName);
        SmartDashboard.putNumber("Turret/PivotFieldX", turretPivotFieldPosition.getX());
        SmartDashboard.putNumber("Turret/PivotFieldY", turretPivotFieldPosition.getY());
        SmartDashboard.putNumber("Turret/HubX", selectedHubPoint.getX());
        SmartDashboard.putNumber("Turret/HubY", selectedHubPoint.getY());
    }
}

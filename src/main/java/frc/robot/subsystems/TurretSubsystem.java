package frc.robot.subsystems;
import java.util.function.BooleanSupplier;
import java.util.stream.DoubleStream;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.util.TurretMath;
import frc.robot.util.ConfigurationHealth;
import frc.robot.util.TurretDiagnostics;
import static edu.wpi.first.units.Units.*;

public class TurretSubsystem extends SubsystemBase {
    private final TalonFX turretKraken = new TalonFX(Constants.Turret.TURRET_KRAKEN_ID, "CANivore1");

    private final MotionMagicVoltage mmPos = new MotionMagicVoltage(0.0);
    private final VoltageOut openLoopVolts = new VoltageOut(0.0);

    // private final StatusSignal<Current> statorCurrentSig = turretKraken.getStatorCurrent();
    // private final StatusSignal<AngularVelocity> velocitySig = turretKraken.getVelocity();

    private double targetTurretRot = 0.0;
    private final Timer stallTimer = new Timer();

    private boolean positionMode = true;
    private boolean autoAimActive = false;
    private boolean visionRecentWarningState = false;
    private boolean inAllianceZone = false;
    private boolean inNeutralZone = false;
    private boolean visionHealthy = false;
    private boolean autoAimSuppressed = false;
    private boolean goalClampedBySoftwareLimit = false;
    private boolean movingShotSolutionValid = false;
    private boolean mappedHoodPositionAvailable = false;
    private boolean mappedRollerRpsAvailable = false;

    private Rotation2d desiredTurretAngle = Rotation2d.kZero;
    private Rotation2d desiredFieldYaw = Rotation2d.kZero;
    private Translation2d turretPivotFieldPosition = Translation2d.kZero;
    private Translation2d selectedTargetPoint = Translation2d.kZero;
    private String allianceName = "Unknown";
    private String targetMode = "HOLD";
    private double recommendedLaunchPitchDeg = Double.NaN;
    private double requiredExitSpeedMps = Double.NaN;
    private double flightTimeSec = Double.NaN;
    private double leadYawDeg = Double.NaN;
    private double mappedHoodPosition = Double.NaN;
    private double mappedRollerRps = Double.NaN;
    private String shotModelName = "Legacy";
    private String robustShotReason = "not-running";
    private double robustDistanceMeters = Double.NaN;
    private double robustRadialVelocityMps = Double.NaN;
    private double robustTangentialVelocityMps = Double.NaN;
    private double robustMargin = Double.NaN;
    private double robustFitError = Double.NaN;
    private double robustReleaseLatencySec = Double.NaN;
    private double robustTranslationScale = 1.0;
    private double robustRotationScale = 1.0;
    private double robustSolverDurationSec = Double.NaN;
    private final boolean configured;
    private double lastAimEvaluationTimestampSec = Double.NaN;
    private final TurretDiagnostics diagnostics = new TurretDiagnostics(
        Constants.Turret.DIAGNOSTIC_TOLERANCE_DEG,
        Constants.Turret.STEP_SETTLE_TIME_SEC,
        Constants.Turret.STEP_ERROR_DEADBAND_DEG,
        Constants.Turret.TRACKING_WINDOW_SEC
    );
    private TurretDiagnostics.Snapshot diagnosticSnapshot;

    private BooleanSupplier shooterReadySupplier = () -> true;

    public TurretSubsystem() {
        TalonFXConfiguration cfg = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(Constants.Turret.STATOR_CURRENT_LIMIT_A))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(Amps.of(Constants.Turret.SUPPLY_CURRENT_LIMIT_A))
                .withSupplyCurrentLimitEnable(true));

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

        var softLimitConfigs = cfg.SoftwareLimitSwitch;
        softLimitConfigs.ReverseSoftLimitEnable = true;
        softLimitConfigs.ReverseSoftLimitThreshold = Constants.Turret.ABSOLUTE_TALON_SOFT_LIMIT_LOW;
        softLimitConfigs.ForwardSoftLimitEnable = true;
        softLimitConfigs.ForwardSoftLimitThreshold = Constants.Turret.ABSOLUTE_TALON_SOFT_LIMIT_HIGH;

        configured = ConfigurationHealth.apply(
            "turret-" + Constants.Turret.TURRET_KRAKEN_ID,
            () -> turretKraken.getConfigurator().apply(cfg)
        );
        turretKraken.getPosition().setUpdateFrequency(100);
        turretKraken.getVelocity().setUpdateFrequency(100);
        turretKraken.getStatorCurrent().setUpdateFrequency(100);
        turretKraken.getMotorVoltage().setUpdateFrequency(Constants.PowerLogging.MOTOR_ELECTRICAL_RATE_HZ);

        zeroEncoder();
        // targetTurretRot = getTurretPositionRotations();
        stallTimer.stop();
        stallTimer.reset();
        setPositionMode(true);

        SmartDashboard.putNumber("Turret kP", slot0Configs.kP);
        SmartDashboard.putNumber("Turret kI", slot0Configs.kI);
        SmartDashboard.putNumber("Turret kD", slot0Configs.kD);
        SmartDashboard.putNumber("Turret Cruise Velocity", motionMagicConfigs.MotionMagicCruiseVelocity);
        SmartDashboard.putNumber("Turret Acceleration", motionMagicConfigs.MotionMagicAcceleration);
        SmartDashboard.putNumber("Turret Jerk", motionMagicConfigs.MotionMagicJerk);
        SmartDashboard.putData("Turret/TestStepPositive15Deg", createStepTrialCommand(15.0));
        SmartDashboard.putData("Turret/TestStepNegative15Deg", createStepTrialCommand(-15.0));
        SmartDashboard.putData("Turret/ApplyTestTuning", Commands.runOnce(this::applyTestTuningFromDashboard, this));
    }

    public TalonFX getKraken() {
        return turretKraken;
    }

    public void stopMotor() {
        turretKraken.stopMotor();
    }

    public boolean hitSoftLimHigh() {
        return getTurretPositionRotations() >= (Constants.Turret.SOFT_LIMIT_HIGH - Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    public boolean hitSoftLimLow() {
        return getTurretPositionRotations() <= (Constants.Turret.SOFT_LIMIT_LOW + Constants.Turret.SOFT_LIMIT_THRESHOLD);
    }

    private double getRawMotorPositionRotations() {
        return turretKraken.getPosition().getValueAsDouble();
    }

    private double getMotorPositionRelativeToForwardRotations() {
        return getRawMotorPositionRotations() - Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS;
    }

    private double convertMotorRotationsToLogicalTurretRotations(double motorRotations) {
        return TurretMath.convertMotorRotationsToTurretRotations(
            motorRotations * Constants.Turret.POSITIVE_TURRET_YAW_MOTOR_SIGN,
            Constants.Turret.MOTOR_ROTATIONS_PER_TURRET_ROTATION
        );
    }

    private double convertLogicalTurretRotationsToMotorRotations(double turretRotations) {
        return TurretMath.convertTurretRotationsToMotorRotations(
            turretRotations,
            Constants.Turret.MOTOR_ROTATIONS_PER_TURRET_ROTATION
        ) * Constants.Turret.POSITIVE_TURRET_YAW_MOTOR_SIGN;
    }

    public double getTurretPositionRotations() {
        return convertMotorRotationsToLogicalTurretRotations(getMotorPositionRelativeToForwardRotations());
    }

    public double getVelocityRps() {
        return convertMotorRotationsToLogicalTurretRotations(turretKraken.getVelocity().getValueAsDouble());
    }

    public Rotation2d getTurretAngle() {
        return Rotation2d.fromRotations(getTurretPositionRotations());
    }

    public Rotation2d getTurretGoal() {
        return Rotation2d.fromRotations(targetTurretRot);
    }

    public double getTargetMotorRotations() {
        return convertLogicalTurretRotationsToMotorRotations(targetTurretRot)
            + Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS;
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
        var positionSignal = turretKraken.getPosition();
        return positionSignal.getStatus().isOK()
            && positionSignal.getTimestamp().isValid()
            && positionSignal.getTimestamp().getLatency() <= Constants.Shooter.MAX_MECHANISM_FEEDBACK_AGE_SEC
            && Math.abs(getTurretAimErrorDeg()) <= Constants.Turret.AIM_TOLERANCE_DEG;
    }

    public boolean isTurretAtAimGoal() {
        return atGoal();
    }

    public void setTargetTurretRotation(double posRot) {
        if (!Double.isFinite(posRot)) {
            movingShotSolutionValid = false;
            robustShotReason = "nonfinite-turret-goal";
            return;
        }
        positionMode = true;
        targetTurretRot = Math.max(Constants.Turret.SOFT_LIMIT_LOW, Math.min(posRot, Constants.Turret.SOFT_LIMIT_HIGH));
    }

    public double getTargetTurretRotation() {
        return targetTurretRot;
    }

    public void setTurretGoal(Rotation2d goal) {
        if (goal == null || !Double.isFinite(goal.getRotations())) {
            movingShotSolutionValid = false;
            robustShotReason = "nonfinite-turret-goal";
            return;
        }
        Rotation2d clampedGoal = wrapTurretAngleToLegalRange(goal);
        desiredTurretAngle = goal;
        goalClampedBySoftwareLimit = Math.abs(clampedGoal.minus(goal).getDegrees()) > 1e-3;
        setTargetTurretRotation(clampedGoal.getRotations());
    }

    public Rotation2d wrapTurretAngleToLegalRange(Rotation2d requested) {
        double legalRot = TurretMath.getNearestEquivalentLegalTurretRotation(
            requested.getRotations(),
            getTurretPositionRotations(),
            Constants.Turret.SOFT_LIMIT_LOW,
            Constants.Turret.SOFT_LIMIT_HIGH
        );
        return Rotation2d.fromRotations(legalRot);
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
            turretKraken.setControl(new VoltageOut(0));
        }
    }

    public boolean getPositionMode() {
        return positionMode;
    }

    public void setVoltage(double output) {
        if (!positionMode && configured && Double.isFinite(output)) {
            turretKraken.setControl(openLoopVolts.withOutput(output));
        }
    }

    public void zeroEncoder() {
        turretKraken.setPosition(Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS);
        setTurretGoal(Rotation2d.kZero);
    }

    public void zeroEncoderAtHardStop() {
        setVoltage(0.0);
        zeroEncoder();
    }

    public void togglePositionMode() {

        if (positionMode && configured) {

            positionMode = false;
        } else {

            positionMode = true;
        }
    }

    public boolean isStallDetected() {
        StatusSignal<Current> statorCurrentSig = turretKraken.getStatorCurrent();
        StatusSignal<AngularVelocity> velocitySig = turretKraken.getVelocity();

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

    public boolean isVisionRecent() {
        return visionRecentWarningState;
    }

    public boolean isOnSoftwareLimit() {
        return goalClampedBySoftwareLimit || hitSoftLimHigh() || hitSoftLimLow();
    }

    public boolean isShotReady() {
        return autoAimActive
            && movingShotSolutionValid
            && isTurretAtAimGoal()
            && !isOnSoftwareLimit()
            && mappedHoodPositionAvailable
            && mappedRollerRpsAvailable
            && shooterReadySupplier.getAsBoolean();
    }

    public boolean isShotSolutionValid() {
        return movingShotSolutionValid;
    }

    public double getLastAimEvaluationTimestampSec() {
        return lastAimEvaluationTimestampSec;
    }

    public boolean hasFiniteMappedSetpoints() {
        return mappedHoodPositionAvailable
            && mappedRollerRpsAvailable
            && Double.isFinite(mappedHoodPosition)
            && Double.isFinite(mappedRollerRps);
    }

    /** Actual bounded actuator goal remains usable for manual feed even without a solver result. */
    public boolean hasFiniteBoundedSetpoint() {
        return Double.isFinite(targetTurretRot)
            && targetTurretRot >= Constants.Turret.SOFT_LIMIT_LOW
            && targetTurretRot <= Constants.Turret.SOFT_LIMIT_HIGH;
    }

    public String getShotReason() {
        return robustShotReason;
    }

    public String getShotModelName() { return shotModelName; }
    public String getTargetModeName() { return targetMode; }

    public void invalidateAimState(String reason) {
        autoAimActive = false;
        movingShotSolutionValid = false;
        mappedHoodPositionAvailable = false;
        mappedRollerRpsAvailable = false;
        robustShotReason = reason;
        lastAimEvaluationTimestampSec = Timer.getFPGATimestamp();
    }

    public Command createStepTrialCommand(double deltaDeg) {
        return Commands.startEnd(
            () -> {
                if (!DriverStation.isTestEnabled()) return;
                autoAimSuppressed = true;
                double startDeg = getTurretAngle().getDegrees();
                Rotation2d requested = Rotation2d.fromDegrees(startDeg + deltaDeg);
                Rotation2d legal = wrapTurretAngleToLegalRange(requested);
                diagnostics.beginStep(Timer.getFPGATimestamp(), startDeg, legal.getDegrees());
                setTurretGoal(legal);
            },
            () -> {
                diagnostics.endStep();
                holdCurrentPosition();
                autoAimSuppressed = false;
            },
            this
        ).onlyWhile(DriverStation::isTestEnabled);
    }

    private void applyTestTuningFromDashboard() {
        if (!DriverStation.isTestEnabled()) {
            SmartDashboard.putString("Turret/TuningApplyStatus", "rejected-outside-test-mode");
            return;
        }
        double kP = SmartDashboard.getNumber("Turret kP", Constants.Turret.KP);
        double kI = SmartDashboard.getNumber("Turret kI", Constants.Turret.KI);
        double kD = SmartDashboard.getNumber("Turret kD", Constants.Turret.KD);
        double cruise = SmartDashboard.getNumber("Turret Cruise Velocity", Constants.Turret.CRUISE_VELOCITY);
        double acceleration = SmartDashboard.getNumber("Turret Acceleration", Constants.Turret.ACCELERATION);
        double jerk = SmartDashboard.getNumber("Turret Jerk", Constants.Turret.JERK);
        if (!DoubleStream.of(kP, kI, kD, cruise, acceleration, jerk).allMatch(Double::isFinite)
            || kP < 0 || kI < 0 || kD < 0 || cruise <= 0 || acceleration <= 0 || jerk <= 0) {
            SmartDashboard.putString("Turret/TuningApplyStatus", "rejected-invalid-values");
            return;
        }
        Slot0Configs gains = new Slot0Configs()
            .withKS(Constants.Turret.KS).withKV(Constants.Turret.KV).withKA(Constants.Turret.KA)
            .withKP(kP).withKI(kI).withKD(kD);
        MotionMagicConfigs motion = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(cruise)
            .withMotionMagicAcceleration(acceleration)
            .withMotionMagicJerk(jerk);
        boolean gainsApplied = ConfigurationHealth.apply(
            "turret-test-gains-" + Constants.Turret.TURRET_KRAKEN_ID,
            () -> turretKraken.getConfigurator().apply(gains)
        );
        boolean motionApplied = ConfigurationHealth.apply(
            "turret-test-motion-" + Constants.Turret.TURRET_KRAKEN_ID,
            () -> turretKraken.getConfigurator().apply(motion)
        );
        SmartDashboard.putString("Turret/TuningApplyStatus", gainsApplied && motionApplied ? "applied" : "failed");
    }

    public void updateAimState(
        String targetMode,
        String allianceName,
        boolean inAllianceZone,
        boolean inNeutralZone,
        boolean visionHealthy,
        boolean visionRecent,
        boolean autoAimActive,
        Translation2d turretPivotFieldPosition,
        Translation2d selectedTargetPoint,
        Rotation2d desiredTurretAngle,
        Rotation2d desiredFieldYaw,
        boolean movingShotSolutionValid,
        double recommendedLaunchPitchDeg,
        double requiredExitSpeedMps,
        double flightTimeSec,
        double leadYawDeg,
        boolean mappedHoodPositionAvailable,
        double mappedHoodPosition,
        boolean mappedRollerRpsAvailable,
        double mappedRollerRps
    ) {
        this.targetMode = targetMode;
        this.allianceName = allianceName;
        this.inAllianceZone = inAllianceZone;
        this.inNeutralZone = inNeutralZone;
        this.visionHealthy = visionHealthy;
        this.visionRecentWarningState = visionRecent;
        this.autoAimActive = autoAimActive;
        this.turretPivotFieldPosition = turretPivotFieldPosition;
        this.selectedTargetPoint = selectedTargetPoint;
        this.desiredTurretAngle = desiredTurretAngle;
        this.desiredFieldYaw = desiredFieldYaw;
        this.movingShotSolutionValid = movingShotSolutionValid;
        this.recommendedLaunchPitchDeg = recommendedLaunchPitchDeg;
        this.requiredExitSpeedMps = requiredExitSpeedMps;
        this.flightTimeSec = flightTimeSec;
        this.leadYawDeg = leadYawDeg;
        this.mappedHoodPositionAvailable = mappedHoodPositionAvailable;
        this.mappedHoodPosition = mappedHoodPosition;
        this.mappedRollerRpsAvailable = mappedRollerRpsAvailable;
        this.mappedRollerRps = mappedRollerRps;
        this.lastAimEvaluationTimestampSec = Timer.getFPGATimestamp();
    }

    public void updateRobustShotState(
        String shotModelName,
        String robustShotReason,
        double robustDistanceMeters,
        double robustRadialVelocityMps,
        double robustTangentialVelocityMps,
        double robustMargin,
        double robustFitError,
        double robustReleaseLatencySec,
        double robustTranslationScale,
        double robustRotationScale,
        double robustSolverDurationSec
    ) {
        this.shotModelName = shotModelName;
        this.robustShotReason = robustShotReason;
        this.robustDistanceMeters = robustDistanceMeters;
        this.robustRadialVelocityMps = robustRadialVelocityMps;
        this.robustTangentialVelocityMps = robustTangentialVelocityMps;
        this.robustMargin = robustMargin;
        this.robustFitError = robustFitError;
        this.robustReleaseLatencySec = robustReleaseLatencySec;
        this.robustTranslationScale = robustTranslationScale;
        this.robustRotationScale = robustRotationScale;
        this.robustSolverDurationSec = robustSolverDurationSec;
    }



    @Override
    public void periodic() {
        if (positionMode && configured) {
            turretKraken.setControl(
                mmPos.withPosition(
                    convertLogicalTurretRotationsToMotorRotations(targetTurretRot)
                        + Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS
                )
            );
        } else {

            turretKraken.setControl(new VoltageOut(0));
        }

        double profilePositionDeg = convertMotorRotationsToLogicalTurretRotations(
            turretKraken.getClosedLoopReference().getValueAsDouble()
                - Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS
        ) * 360.0;
        diagnosticSnapshot = diagnostics.update(
            Timer.getFPGATimestamp(),
            getTurretGoal().getDegrees(),
            profilePositionDeg,
            getTurretAngle().getDegrees()
        );

        // for tuning pid


        publishTelemetry();
    }

    private void publishTelemetry() {
        SmartDashboard.putNumber("Turret/ActualAngleDeg", getTurretAngle().getDegrees());
        SmartDashboard.putNumber("Turret/GoalAngleDeg", getTurretGoal().getDegrees());
        SmartDashboard.putNumber("Turret/DesiredAngleDeg", desiredTurretAngle.getDegrees());
        SmartDashboard.putNumber("Turret/DesiredFieldYawDeg", desiredFieldYaw.getDegrees());
        SmartDashboard.putNumber("Turret/RawMotorRot", getRawMotorPositionRotations());
        SmartDashboard.putNumber("Turret/RelativeMotorRot", getMotorPositionRelativeToForwardRotations());
        SmartDashboard.putNumber("Turret/GoalMotorRot", getTargetMotorRotations());
        SmartDashboard.putNumber("Turret/ForwardReferenceMotorRot", Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS);
        SmartDashboard.putNumber("Turret/AbsSoftLimitLowMotorRot", Constants.Turret.ABSOLUTE_TALON_SOFT_LIMIT_LOW);
        SmartDashboard.putNumber("Turret/AbsSoftLimitHighMotorRot", Constants.Turret.ABSOLUTE_TALON_SOFT_LIMIT_HIGH);
        SmartDashboard.putNumber("Turret/AimErrorDeg", getTurretAimErrorDeg());
        SmartDashboard.putBoolean("Turret/AtAimGoal", isTurretAtAimGoal());
        SmartDashboard.putBoolean("Turret/ShotReady", isShotReady());
        SmartDashboard.putBoolean("Turret/AutoAimActive", autoAimActive);
        SmartDashboard.putBoolean("Turret/AutoAimSuppressed", autoAimSuppressed);
        SmartDashboard.putBoolean("Turret/InAllianceZone", inAllianceZone);
        SmartDashboard.putBoolean("Turret/InNeutralZone", inNeutralZone);
        SmartDashboard.putBoolean("Turret/VisionHealthy", visionHealthy);
        SmartDashboard.putBoolean("Turret/VisionRecentWarningState", visionRecentWarningState);
        SmartDashboard.putBoolean("Turret/VisionAgeGatesFeed", false);
        SmartDashboard.putBoolean("Turret/OnSoftwareLimit", isOnSoftwareLimit());
        SmartDashboard.putBoolean("Turret/MovingShotSolutionValid", movingShotSolutionValid);
        SmartDashboard.putBoolean("Turret/MappedHoodPositionAvailable", mappedHoodPositionAvailable);
        SmartDashboard.putBoolean("Turret/MappedRollerRpsAvailable", mappedRollerRpsAvailable);
        SmartDashboard.putString("Turret/Alliance", allianceName);
        SmartDashboard.putString("Turret/TargetMode", targetMode);
        SmartDashboard.putNumber("Turret/PivotFieldX", turretPivotFieldPosition.getX());
        SmartDashboard.putNumber("Turret/PivotFieldY", turretPivotFieldPosition.getY());
        SmartDashboard.putNumber("Turret/TargetX", selectedTargetPoint.getX());
        SmartDashboard.putNumber("Turret/TargetY", selectedTargetPoint.getY());
        SmartDashboard.putNumber("Turret/HubX", selectedTargetPoint.getX());
        SmartDashboard.putNumber("Turret/HubY", selectedTargetPoint.getY());
        SmartDashboard.putNumber("Turret/RecommendedLaunchPitchDeg", recommendedLaunchPitchDeg);
        SmartDashboard.putNumber("Turret/RequiredExitSpeedMps", requiredExitSpeedMps);
        SmartDashboard.putNumber("Turret/FlightTimeSec", flightTimeSec);
        SmartDashboard.putNumber("Turret/LeadYawDeg", leadYawDeg);
        SmartDashboard.putNumber("Turret/MappedHoodPosition", mappedHoodPosition);
        SmartDashboard.putNumber("Turret/MappedRollerRps", mappedRollerRps);
        SmartDashboard.putString("Turret/ShotModel", shotModelName);
        SmartDashboard.putString("Turret/RobustShotReason", robustShotReason);
        SmartDashboard.putNumber("Turret/RobustDistanceMeters", robustDistanceMeters);
        SmartDashboard.putNumber("Turret/RobustRadialVelocityMps", robustRadialVelocityMps);
        SmartDashboard.putNumber("Turret/RobustTangentialVelocityMps", robustTangentialVelocityMps);
        SmartDashboard.putNumber("Turret/RobustMargin", robustMargin);
        SmartDashboard.putNumber("Turret/RobustFitError", robustFitError);
        SmartDashboard.putNumber("Turret/RobustReleaseLatencySec", robustReleaseLatencySec);
        SmartDashboard.putNumber("Turret/RobustTranslationScale", robustTranslationScale);
        SmartDashboard.putNumber("Turret/RobustRotationScale", robustRotationScale);
        SmartDashboard.putNumber("Turret/RobustSolverDurationMs", robustSolverDurationSec * 1000.0);
        SmartDashboard.putBoolean("Turret/Kaden", positionMode);
        if (diagnosticSnapshot != null) {
            SmartDashboard.putNumber("Turret/Diagnostics/ProfilePositionDeg",
                convertMotorRotationsToLogicalTurretRotations(
                    turretKraken.getClosedLoopReference().getValueAsDouble()
                        - Constants.Turret.FORWARD_REFERENCE_MOTOR_ROTATIONS
                ) * 360.0);
            SmartDashboard.putNumber("Turret/Diagnostics/ProfileVelocityDegPerSec",
                convertMotorRotationsToLogicalTurretRotations(
                    turretKraken.getClosedLoopReferenceSlope().getValueAsDouble()
                ) * 360.0);
            SmartDashboard.putNumber("Turret/Diagnostics/MeasuredVelocityDegPerSec", getVelocityRps() * 360.0);
            SmartDashboard.putNumber("Turret/Diagnostics/GoalErrorDeg", diagnosticSnapshot.goalErrorDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/ProfileTrackingErrorDeg", diagnosticSnapshot.profileTrackingErrorDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/OutputVoltageV", turretKraken.getMotorVoltage().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Diagnostics/SupplyCurrentA", turretKraken.getSupplyCurrent().getValueAsDouble());
            SmartDashboard.putNumber("Turret/Diagnostics/StatorCurrentA", turretKraken.getStatorCurrent().getValueAsDouble());
            boolean supplyLimit = Boolean.TRUE.equals(turretKraken.getFault_SupplyCurrLimit().getValue());
            boolean statorLimit = Boolean.TRUE.equals(turretKraken.getFault_StatorCurrLimit().getValue());
            boolean suspectedLimit = !supplyLimit && !statorLimit
                && Math.abs(turretKraken.getMotorVoltage().getValueAsDouble()) >= 11.5
                && Math.abs(diagnosticSnapshot.profileTrackingErrorDeg()) > Constants.Turret.AIM_TOLERANCE_DEG;
            SmartDashboard.putString("Turret/Diagnostics/CurrentLimitStatus",
                supplyLimit || statorLimit ? "active-reported" : (suspectedLimit ? "suspected" : "not-observed"));
            SmartDashboard.putNumber("Turret/Diagnostics/StepSettlingTimeSec", diagnosticSnapshot.settlingTimeSec());
            SmartDashboard.putNumber("Turret/Diagnostics/StepOvershootDeg", diagnosticSnapshot.overshootDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/StepFinalBiasDeg", diagnosticSnapshot.finalBiasDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/StepErrorSignChanges", diagnosticSnapshot.errorSignChanges());
            SmartDashboard.putNumber("Turret/Diagnostics/RollingGoalRmsDeg", diagnosticSnapshot.rollingGoalErrorRmsDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/RollingGoalPeakDeg", diagnosticSnapshot.rollingGoalErrorPeakDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/RollingProfileRmsDeg", diagnosticSnapshot.rollingProfileErrorRmsDeg());
            SmartDashboard.putNumber("Turret/Diagnostics/RollingProfilePeakDeg", diagnosticSnapshot.rollingProfileErrorPeakDeg());
        }
    }
}

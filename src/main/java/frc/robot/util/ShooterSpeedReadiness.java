package frc.robot.util;

/** Signed flywheel minimum-speed readiness with separate entry and running tolerances. */
public final class ShooterSpeedReadiness {
    private final double startToleranceRps;
    private final double startToleranceScale;
    private final double runningToleranceRps;
    private final double runningToleranceScale;
    private boolean ready;
    private double lastTargetRps = Double.NaN;

    public ShooterSpeedReadiness(double startToleranceRps, double startToleranceScale,
        double runningToleranceRps, double runningToleranceScale) {
        this.startToleranceRps = startToleranceRps;
        this.startToleranceScale = startToleranceScale;
        this.runningToleranceRps = runningToleranceRps;
        this.runningToleranceScale = runningToleranceScale;
    }

    public boolean update(double actualRps, double targetRps, boolean feedbackUsable) {
        if (!feedbackUsable || !Double.isFinite(actualRps) || !Double.isFinite(targetRps)
            || targetRps == 0.0 || Math.signum(actualRps) != Math.signum(targetRps)) {
            reset();
            return false;
        }
        double startBand = Math.max(startToleranceRps, Math.abs(targetRps) * startToleranceScale);
        // A new calibration setting or a large aiming change must enter the start band again.
        if (!Double.isFinite(lastTargetRps) || Math.abs(targetRps - lastTargetRps) > startBand) ready = false;
        double band = ready
            ? Math.max(runningToleranceRps, Math.abs(targetRps) * runningToleranceScale) : startBand;
        // Same-direction overspeed never blocks feed; only insufficient speed does.
        ready = Math.abs(actualRps) >= Math.max(0.0, Math.abs(targetRps) - band);
        lastTargetRps = targetRps;
        return ready;
    }

    public void reset() {
        ready = false;
        lastTargetRps = Double.NaN;
    }
}

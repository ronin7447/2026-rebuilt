package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ShootingDriveLimiterTest {
    private ChassisSpeeds calculate(ShootingDriveLimiter limiter, ChassisSpeeds request, double time) {
        return limiter.calculate(request, true, time, 1.0, 0.8, 1.0, 2.0);
    }

    @Test
    void diagonalCommandRespectsVectorSpeedAndAccelerationCaps() {
        var limiter = new ShootingDriveLimiter();
        limiter.reset(new ChassisSpeeds(), 0);
        ChassisSpeeds previous = new ChassisSpeeds();
        for (int i = 1; i <= 100; i++) {
            var result = calculate(limiter, new ChassisSpeeds(1, 1, 1), i * 0.02);
            assertTrue(Math.hypot(result.vxMetersPerSecond, result.vyMetersPerSecond) <= 1 + 1e-9);
            assertTrue(Math.hypot(result.vxMetersPerSecond - previous.vxMetersPerSecond,
                result.vyMetersPerSecond - previous.vyMetersPerSecond) <= 0.02 + 1e-9);
            assertTrue(Math.abs(result.omegaRadiansPerSecond - previous.omegaRadiansPerSecond) <= 0.04 + 1e-9);
            previous = result;
        }
        assertEquals(1, Math.hypot(previous.vxMetersPerSecond, previous.vyMetersPerSecond), 1e-9);
        assertEquals(0.8, previous.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void enteringShootingRampsSpeedButReversesDirectionImmediately() {
        var limiter = new ShootingDriveLimiter();
        limiter.calculate(new ChassisSpeeds(3, 0, 2), false, 1, 1, 0.8, 1, 2);
        var result = calculate(limiter, new ChassisSpeeds(-1, 0, -0.8), 1.02);
        assertEquals(-2.98, result.vxMetersPerSecond, 1e-9);
        assertEquals(1.96, result.omegaRadiansPerSecond, 1e-9);
        // Deceleration takes time; readiness checks actual motion throughout this transition.
        assertEquals(-2.96, calculate(limiter, new ChassisSpeeds(), 5).vxMetersPerSecond, 1e-9);
        assertEquals(-2.96, calculate(limiter, new ChassisSpeeds(), 4).vxMetersPerSecond, 1e-9);
    }

    @Test
    void steeringRespondsImmediatelyWhileSpeedStillRamps() {
        var limiter = new ShootingDriveLimiter();
        limiter.reset(new ChassisSpeeds(0.5, 0, 0), 0);
        var turn = calculate(limiter, new ChassisSpeeds(0, 1, 0), 0.02);
        assertEquals(0, turn.vxMetersPerSecond, 1e-9);
        assertEquals(0.52, turn.vyMetersPerSecond, 1e-9);
        var reverse = calculate(limiter, new ChassisSpeeds(0, -0.52, 0), 0.04);
        assertEquals(0, reverse.vxMetersPerSecond, 1e-9);
        assertEquals(-0.52, reverse.vyMetersPerSecond, 1e-9);
    }

    @Test
    void releasingStickRampsToRestWithoutChangingDirection() {
        var limiter = new ShootingDriveLimiter();
        limiter.reset(new ChassisSpeeds(0.03, 0.04, 0), 0);
        var slowing = calculate(limiter, new ChassisSpeeds(), 0.02);
        assertEquals(0.018, slowing.vxMetersPerSecond, 1e-9);
        assertEquals(0.024, slowing.vyMetersPerSecond, 1e-9);
        calculate(limiter, new ChassisSpeeds(), 0.04);
        var stopped = calculate(limiter, new ChassisSpeeds(), 0.06);
        assertEquals(0, stopped.vxMetersPerSecond, 1e-9);
        assertEquals(0, stopped.vyMetersPerSecond, 1e-9);
        var resting = calculate(limiter, new ChassisSpeeds(), 0.08);
        assertEquals(0, resting.vxMetersPerSecond, 1e-9);
        assertEquals(0, resting.vyMetersPerSecond, 1e-9);
    }

    @Test
    void ordinaryDrivingReturnsRequestedMotionImmediately() {
        var limiter = new ShootingDriveLimiter();
        var result = limiter.calculate(new ChassisSpeeds(3, -2, 4), false, 1, 1, 0.8, 1, 2);
        assertEquals(3, result.vxMetersPerSecond);
        assertEquals(-2, result.vyMetersPerSecond);
        assertEquals(4, result.omegaRadiansPerSecond);
    }
}

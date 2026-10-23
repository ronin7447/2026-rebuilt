package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Optional;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class StableShotSolutionSelectorTest {
    private RobustShotRequest request(double sideSpeed, double pitchDeg, double targetX) {
        var speeds = new ChassisSpeeds(0, sideSpeed, 0);
        var motion = new ShotMotionState(new Pose2d(), speeds, speeds, 0, 0, 0,
            new Rotation3d(0, Math.toRadians(pitchDeg), 0));
        return new RobustShotRequest(motion, new Translation2d(), 0.4064,
            new Translation3d(targetX, 0, 2.13), Rotation2d.kZero,
            -2.266942, -0.631116, 11.120291, 0.14, ShotTargetConstraints.hub(2.13));
    }

    @Test
    void revalidationUsesCurrentMotionAndTiltWithoutRenewingCacheAge() {
        var selector = new StableShotSolutionSelector(0.10);
        var initial = request(0, 8, 4);
        var fresh = RobustShotModel.solve(initial);
        assertTrue(fresh.orElseThrow().quality().insideTolerance());
        selector.select(initial, fresh, 0, true);
        var moved = request(0.2, 0, 4);
        var reused = selector.select(moved, Optional.empty(), 0.08, true).orElseThrow();
        assertTrue(reused.quality().insideTolerance(), reused.quality().reason());
        assertEquals(CommandSource.REVALIDATED_PREVIOUS, reused.commandSource());
        assertEquals(fresh.get().rollerRps(), reused.rollerRps());
        assertTrue(Math.abs(reused.desiredTurretYaw().getDegrees()) > 0.1);
        assertTrue(Math.abs(reused.hoodPositionRot() - fresh.get().hoodPositionRot()) > 0.2);
        assertEquals(RobustShotModel.solve(request(0, 0, 4)).orElseThrow().hoodPositionRot(),
            reused.hoodPositionRot(), 1e-9);
        assertTrue(selector.select(moved, Optional.empty(), 0.101, true).isEmpty());
    }

    @Test
    void invalidRevalidationCannotMasqueradeAsReadyAndFreshValidShotsWin() {
        var selector = new StableShotSolutionSelector(0.10);
        var initial = request(0, 0, 4);
        var fresh = RobustShotModel.solve(initial);
        selector.select(initial, fresh, 0, true);
        assertTrue(selector.select(request(20, 0, 4), Optional.empty(), 0.02, true).isEmpty());
        var nextRequest = request(0.1, 0, 4);
        var next = RobustShotModel.solve(nextRequest);
        assertSame(next.get(), selector.select(nextRequest, next, 0.04, true).orElseThrow());
    }

    @Test
    void previousShotMustStillPassCurrentTiltLimitsAndHubEntryGeometry() {
        var initial = request(0, 0, 4);
        var previous = RobustShotModel.solve(initial).orElseThrow();
        var tilted = RobustShotModel.revalidate(request(0, 30, 4), previous).orElseThrow();
        assertFalse(tilted.quality().insideTolerance());
        assertEquals("tilt-corrected-hood-out-of-range", tilted.quality().reason());
        var c = initial.targetConstraints();
        var steepEntry = new ShotTargetConstraints(c.targetType(), c.targetHeightMeters(), c.goalRadiusMeters(),
            c.fuelRadiusMeters(), c.hubOpeningDepthMeters(), c.hubFrontLipHeightMeters(), c.hubLipClearanceMeters(),
            89.9, true, c.minDistanceMeters(), c.maxDistanceMeters(), c.minRadialVelocityMps(),
            c.maxRadialVelocityMps(), c.selectionMode());
        var stricter = new RobustShotRequest(initial.motionState(), initial.robotToTurretPivot(),
            initial.shooterExitHeightMeters(), initial.targetFieldPoint(), initial.turretZeroOffset(),
            initial.minHoodPositionRot(), initial.maxHoodPositionRot(), initial.maxExitSpeedMps(),
            initial.releaseLatencySec(), steepEntry);
        var rejected = RobustShotModel.revalidate(stricter, previous).orElseThrow();
        assertFalse(rejected.quality().insideTolerance());
        assertEquals("hub-entry-angle", rejected.quality().reason());
    }

    @Test
    void targetChangeDisableResetAndClockReversalDiscardCache() {
        var selector = new StableShotSolutionSelector(0.10);
        var initial = request(0, 0, 4);
        var fresh = RobustShotModel.solve(initial);
        selector.select(initial, fresh, 1, true);
        assertTrue(selector.select(request(0, 0, 4.01), Optional.empty(), 1.02, true).isEmpty());
        selector.select(initial, fresh, 2, true);
        assertTrue(selector.select(initial, Optional.empty(), 1.99, true).isEmpty());
        selector.select(initial, fresh, 3, true);
        assertTrue(selector.select(initial, Optional.empty(), 3.02, false).isEmpty());
        assertTrue(selector.select(initial, Optional.empty(), 3.04, true).isEmpty());
        selector.select(initial, fresh, 4, true);
        selector.reset();
        assertTrue(selector.select(initial, Optional.empty(), 4.02, true).isEmpty());
    }
}

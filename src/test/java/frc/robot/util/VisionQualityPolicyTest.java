package frc.robot.util;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class VisionQualityPolicyTest {
    private String check(int tags, double distance, double ambiguity) {
        return VisionQualityPolicy.rejectionReason(tags, distance, ambiguity, 4, 6, .2);
    }

    @Test
    void acceptsNearObservationsAndRejectsDistantSingleAndMultiTag() {
        assertEquals("", check(1, 4, .2));
        assertEquals("tag-range", check(1, 4.01, .01));
        assertEquals("", check(2, 6, 0));
        assertEquals("tag-range", check(2, 6.01, 0));
        assertEquals("", check(1, 2, .05));
        assertEquals("", check(4, 2, 0));
    }

    @Test
    void singleTagFallbackUsesSingleTagQualityRegardlessOfDetectionsInFrame() {
        // Caller supplies solver targetsUsed, not total detected targets.
        assertEquals("single-tag-ambiguity", check(1, 3, .21));
        assertEquals("single-tag-ambiguity", check(1, 3, -1));
        assertEquals("single-tag-ambiguity", check(1, 3, Double.NaN));
        assertEquals("invalid-tag-distance", check(0, 3, 0));
        assertEquals("invalid-tag-distance", check(1, Double.POSITIVE_INFINITY, 0));
        assertEquals("invalid-tag-distance", check(1, Double.NaN, 0));
    }

    @Test
    void xyOnlyFusionRemovesVisionYawAuthorityAndCanBeToggledBack() {
        assertEquals(1e6, VisionQualityPolicy.headingStdDev(false, .1));
        assertEquals(.1, VisionQualityPolicy.headingStdDev(true, .1));
        // Scalar steady-state gain approximation: yaw noise cannot meaningfully
        // change heading even with a relatively uncertain 0.1 rad gyro/odometry prior.
        double priorVariance = .1 * .1;
        double visionSigma = VisionQualityPolicy.headingStdDev(false, .1);
        assertTrue(priorVariance / (priorVariance + visionSigma * visionSigma) < 1e-12);
    }
}

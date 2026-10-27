package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

class ReleaseStatePredictorTest {
    @Test
    void predictsAcceleratingTranslationAndRotationAtDeparture() {
        Pose2d pose = new Pose2d(2, 3, Rotation2d.fromRadians(0.4));
        ChassisSpeeds field = new ChassisSpeeds(1, -2, 0.5);
        ShotMotionState motion = new ShotMotionState(pose,
            ChassisSpeeds.fromFieldRelativeSpeeds(field, pose.getRotation()), field,
            2, -1, 3, new Rotation3d(0.1, -0.2, 0.4));
        var release = ReleaseStatePredictor.predict(motion, 0.2);
        assertEquals(2.24, release.robotPose().getX(), 1e-9);
        assertEquals(2.58, release.robotPose().getY(), 1e-9);
        assertEquals(0.56, release.robotPose().getRotation().getRadians(), 1e-9);
        assertEquals(1.4, release.fieldRelativeSpeeds().vxMetersPerSecond, 1e-9);
        assertEquals(-2.2, release.fieldRelativeSpeeds().vyMetersPerSecond, 1e-9);
        assertEquals(1.1, release.fieldRelativeSpeeds().omegaRadiansPerSecond, 1e-9);
        ChassisSpeeds roundTrip = ChassisSpeeds.fromRobotRelativeSpeeds(release.robotRelativeSpeeds(),
            release.robotPose().getRotation());
        assertEquals(1.4, roundTrip.vxMetersPerSecond, 1e-9);
        assertEquals(-2.2, roundTrip.vyMetersPerSecond, 1e-9);
        assertEquals(0.1, release.robotOrientation().getX(), 1e-9);
        assertEquals(-0.2, release.robotOrientation().getY(), 1e-9);
    }
}

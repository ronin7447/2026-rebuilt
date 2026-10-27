package frc.robot.util.sotm;

import static org.junit.jupiter.api.Assertions.*;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.constants.Constants;

class ShotEvaluatorFixtureParityTest {
    @Test
    void javaMatchesVersionedCalculatorFixtures() throws IOException {
        List<String> lines = Files.readAllLines(Path.of("shared/shot-evaluator-fixtures-v1.csv"));
        assertTrue(lines.get(0).startsWith("schema_version,id,target_type"));
        for (String line : lines.subList(1, lines.size())) {
            String[] value = line.split(",", -1);
            String id = value[1];
            ShotTargetConstraints.TargetType type = ShotTargetConstraints.TargetType.valueOf(value[2]);
            double headingRad = Math.toRadians(number(value[5]));
            Pose2d pose = new Pose2d(number(value[3]), number(value[4]), new Rotation2d(headingRad));
            ChassisSpeeds fieldSpeeds = new ChassisSpeeds(number(value[6]), number(value[7]), number(value[8]));
            ChassisSpeeds robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, pose.getRotation());
            ShotMotionState motion = new ShotMotionState(
                pose,
                robotSpeeds,
                fieldSpeeds,
                number(value[9]),
                number(value[10]),
                number(value[11]),
                new Rotation3d(
                    Math.toRadians(number(value[13])),
                    Math.toRadians(number(value[12])),
                    headingRad
                )
            );
            double targetZ = number(value[16]);
            ShotTargetConstraints constraints = type == ShotTargetConstraints.TargetType.HUB
                ? ShotTargetConstraints.hub(targetZ)
                : ShotTargetConstraints.passback(targetZ);
            var result = RobustShotModel.solve(new RobustShotRequest(
                motion,
                Constants.Turret.ROBOT_TO_TURRET_PIVOT,
                Constants.Shooter.EXIT_HEIGHT_METERS,
                new Translation3d(number(value[14]), number(value[15]), targetZ),
                Rotation2d.kZero,
                Constants.Shooter.MIN_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_HOOD_POSITION_ROT,
                Constants.Shooter.MAX_EXIT_SPEED_MPS,
                0.14,
                constraints
            ));
            boolean expectedFeasible = Boolean.parseBoolean(value[17]);
            if (!expectedFeasible && !Double.isFinite(number(value[8]))) {
                assertTrue(result.isEmpty(), id);
                continue;
            }
            assertTrue(result.isPresent(), id);
            assertEquals(expectedFeasible, result.get().trajectoryFeasible(), id);
            assertEquals(CommandSource.valueOf(value[18]), result.get().commandSource(), id);
            assertEquals(number(value[19]), result.get().targetPlaneCrossingTimeSec(), number(value[20]), id);
        }
    }

    private static double number(String value) {
        return Double.parseDouble(value);
    }
}

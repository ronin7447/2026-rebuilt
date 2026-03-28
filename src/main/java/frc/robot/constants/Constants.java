package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class Constants {
    /* Drivebase Constants */
    public static final class Drivebase {
        public static final double MAX_SPEED = 0.3 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DRIVE_DEADBAND = MAX_SPEED * 0.2; // 20% of max speed as deadband
        public static final double ROTATIONAL_DEADBAND = MAX_ANGULAR_RATE * 0.2; // 20% of max angular rate as deadband
    }
    
    public static final class Turret {
        public static final int TURRET_KRAKEN_ID = 37; // Turret X44 Kraken ID
        public static final double STALL_CURRENT_A = 6.0;
        public static final double STALL_VEL_RPS = 0.25;
        public static final double STALL_TIME_S = 0.10;
        public static final double KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double KP = 45.0; // 35 V per rotation of position error
        public static final double KI = 0.00; // no output for integrated error
        public static final double KD = 0.20; // 0.20 V output for error derivative

        public static final double CRUISE_VELOCITY = 100.0;
        public static final double ACCELERATION = 400.0;
        public static final double JERK = 400.0;
        public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 4; // 100:15?? 100;25
        public static final double POSITIVE_TURRET_YAW_MOTOR_SIGN = -1.0;
        public static final Translation2d ROBOT_TO_TURRET_PIVOT = new Translation2d(-0.1651, -0.2159); // TODO: set turret pivot offset in robot coordinates
        public static final double FORWARD_REFERENCE_MOTOR_ROTATIONS = 0.00;
        public static final Rotation2d TURRET_ZERO_OFFSET = Rotation2d.fromDegrees(0.0); // Unused with forward motor reference zeroing
        public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(0.0);
        public static final double AIM_TOLERANCE_DEG = 2.0;
        public static final double MAX_VISION_AGE_FOR_SHOT_SEC = 0.25;
        
        public static final double POSITION_TOLERANCE_ROT = AIM_TOLERANCE_DEG / 360.0;

        // motor soft limits relative to the calibrated forward reference position
        public static final double TALON_SOFT_LIMIT_LOW = -1.90;
        public static final double TALON_SOFT_LIMIT_HIGH = 4.235;
        public static final double ABSOLUTE_TALON_SOFT_LIMIT_LOW = FORWARD_REFERENCE_MOTOR_ROTATIONS + TALON_SOFT_LIMIT_LOW;
        public static final double ABSOLUTE_TALON_SOFT_LIMIT_HIGH = FORWARD_REFERENCE_MOTOR_ROTATIONS + TALON_SOFT_LIMIT_HIGH;

        private static final double LOGICAL_LIMIT_FROM_RAW_LOW =
            (TALON_SOFT_LIMIT_LOW * POSITIVE_TURRET_YAW_MOTOR_SIGN) / MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        private static final double LOGICAL_LIMIT_FROM_RAW_HIGH =
            (TALON_SOFT_LIMIT_HIGH * POSITIVE_TURRET_YAW_MOTOR_SIGN) / MOTOR_ROTATIONS_PER_TURRET_ROTATION;

        public static final double SOFT_LIMIT_LOW = Math.min(LOGICAL_LIMIT_FROM_RAW_LOW, LOGICAL_LIMIT_FROM_RAW_HIGH);
        public static final double SOFT_LIMIT_HIGH = Math.max(LOGICAL_LIMIT_FROM_RAW_LOW, LOGICAL_LIMIT_FROM_RAW_HIGH);
        
        public static final double SOFT_LIMIT_THRESHOLD = Rotation2d.fromDegrees(2.0).getRotations();
        
    }

    public static final class Vision {
        public static final AprilTagFieldLayout FIELD_LAYOUT_OBJECT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
        public static final boolean FORCE_ACCEPT_VISION_MEASUREMENTS = false;

        /* AprilTag Camera 1 */
        public static final String APRILTAG_CAM1 = "APRILTAG_LEFT";
        public static final Transform3d ROBOT_TO_CAM1 = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(1.0),
                Units.inchesToMeters(12.0),
                Units.inchesToMeters(18.625)
            ),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        );

        /* AprilTag Camera 2 */
        public static final String APRILTAG_CAM2 = "APRILTAG_RIGHT";
        public static final Transform3d ROBOT_TO_CAM2 = new Transform3d(
            new Translation3d(
                Units.inchesToMeters(1.0),
                Units.inchesToMeters(-12.0),
                Units.inchesToMeters(17.0)
            ),
            new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0))
        );

        /* Field layout, CA district is welded */
        public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

        public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.60;
        public static final double MAX_POSE_Z_METERS = 2.05;
        public static final double MAX_POSE_ROLL_PITCH_DEG = 45.0;
        public static final double MAX_MEASUREMENT_AGE_SEC = 1.00;
        public static final double VISION_HEALTHY_AGE_SEC = 0.70;
        public static final double MULTI_TAG_BASE_XY_STD_DEV_METERS = 0.20;
        public static final double SINGLE_TAG_BASE_XY_STD_DEV_METERS = 0.50;
        public static final double MULTI_TAG_BASE_THETA_STD_DEV_RAD = Units.degreesToRadians(3.0);
        public static final double SINGLE_TAG_BASE_THETA_STD_DEV_RAD = Units.degreesToRadians(8.0);
        public static final double DISTANCE_STD_DEV_SCALE = 1.0 / 25.0;
    }

    public static final class FieldTargets {
        private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2);
        private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7);
        private static final double ALLIANCE_ZONE_DEPTH_METERS = Units.inchesToMeters(158.6);
        public static final double HUB_TARGET_HEIGHT_METERS = 1.83 + 0.30;

        // 2026 Rebuilt: each ALLIANCE HUB is centered 158.6 in from its alliance wall and centered across field width.
        public static final Translation2d BLUE_HUB_AIM_POINT =
            new Translation2d(ALLIANCE_ZONE_DEPTH_METERS, (FIELD_WIDTH_METERS / 2.0) + 0.00);
        public static final Translation2d RED_HUB_AIM_POINT =
            new Translation2d(FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS, FIELD_WIDTH_METERS / 2.0);

        public static final List<Translation2d> BLUE_ALLIANCE_ZONE = List.of(
            new Translation2d(0.0, 0.0),
            new Translation2d(ALLIANCE_ZONE_DEPTH_METERS, 0.0),
            new Translation2d(ALLIANCE_ZONE_DEPTH_METERS, FIELD_WIDTH_METERS),
            new Translation2d(0.0, FIELD_WIDTH_METERS)
        );

        public static final List<Translation2d> RED_ALLIANCE_ZONE = List.of(
            new Translation2d(FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS, 0.0),
            new Translation2d(FIELD_LENGTH_METERS, 0.0),
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Translation2d(FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS, FIELD_WIDTH_METERS)
        );

        private FieldTargets() {}

        public static Translation2d getHubForAlliance(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RED_HUB_AIM_POINT : BLUE_HUB_AIM_POINT;
        }

        public static Translation3d getHubAimPoint3d(DriverStation.Alliance alliance) {
            Translation2d hubPoint = getHubForAlliance(alliance);
            return new Translation3d(hubPoint.getX(), hubPoint.getY(), HUB_TARGET_HEIGHT_METERS);
        }

        public static List<Translation2d> getAllianceZone(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;
        }
    }

    public static final class Intake {
        public static final int ROLLER_KRAKEN_ID = 40;
        public static final int EXTENSION_KRAKEN_MASTER_ID = 41;
        public static final int EXTENSION_KRAKEN_FOLLOWER_ID = 42;
        public static final double EXTENSION_VELOCITY = 15.0;
        public static final double EXTENSION_ACCELERATION = 30.0;
        public static final double EXTENSION_JERK = 60.0;

        public static final double INTAKE_STALL_CURRENT = 6.0;
        public static final double INTAKE_STALL_VELOCITY = 10;
        public static final double STALL_TIME_S = 0.25;

        public static final double INTAKE_TARGET_ROT_TOLERANCE = 0.02;

        public static final double INTAKE_COLLAPSED_POS = 0.15;
        public static final double INTAKE_EXTENDED_POS = 2.017578125;

        public static final double ROLLER_VELOCITY = 72.0;
    }

    public static final class Shooter {
        public static final int HOOD_KRAKEN_ID = 21;
        public static final int ROLLER_KRAKEN_ID = 31;

        public static final double KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double KP = 35.0; // 35 V per rotation of position error
        public static final double KI = 0.00; // no output for integrated error
        public static final double KD = 0.20; // 0.20 V output for error derivative

        public static final double ROLLER_KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double ROLLER_KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double ROLLER_KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double ROLLER_KP = 0.11; // An error of 1 rps results in 0.11 V output
        public static final double ROLLER_KI = 0.00; // no output for integrated error
        public static final double ROLLER_KD = 0.00; // no output for error derivative
        public static final double ROLLER_DIRECTION_SIGN = -1.0;
        public static final double ROLLER_VELOCITY = -31.98;
        public static final double ROLLER_ACCELERATION = 400.0; // Target acceleration of 400 rps/s
        public static final double ROLLER_JERK = 4000.0; // Target jerk of 4000 rps/s/s

        public static final double ROLLER_TOLERANCE = 1.0; // How much the actual velocity in rps can differ from target
        public static final double EXIT_HEIGHT_METERS = 0.4064;
        public static final double HOOD_POSITION_PER_LAUNCH_DEG_SLOPE = 0.0857913;
        public static final double HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT = -7.49442;
        public static final double MIN_HOOD_POSITION_ROT = -2.266942;
        public static final double MAX_HOOD_POSITION_ROT = -0.631116;
        public static final double FAR_SHOT_DISTANCE_METERS = 4.0;
        public static final double FAR_SHOT_MIN_HOOD_POSITION_ROT = -1.40;
        public static final double MAX_EXIT_SPEED_MPS = 12.0;
        public static final double ROLLER_RPS_PER_EXIT_SPEED_SLOPE = 8.01922;
        public static final double ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT = 10.82394;
    }

    public static final class Outtake {
        public static final double ROLLER_KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double ROLLER_KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double ROLLER_KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double ROLLER_KP = 0.11; // An error of 1 rps results in 0.11 V output
        public static final double ROLLER_KI = 0.00; // no output for integrated error
        public static final double ROLLER_KD = 0.00; // no output for error derivative
        public static final double ROLLER_VELOCITY = -35;
        public static final double ROLLER_ACCELERATION = 400.0; // Target acceleration of 400 rps/s
        public static final double ROLLER_JERK = 4000.0; // Target jerk of 4000 rps/s/s
        public static final int ROLLER_KRAKEN_ID = 25;
    }
    public static final class Indexer {
        public static final double ROLLER_KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double ROLLER_KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double ROLLER_KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double ROLLER_KP = 0.11; // An error of 1 rps results in 0.11 V output
        public static final double ROLLER_KI = 0.00; // no output for integrated error
        public static final double ROLLER_KD = 0.00; // no output for error derivative
        public static final double ROLLER_VELOCITY = -96.0;
        public static final double ROLLER_ACCELERATION = 400.0; // Target acceleration of 400 rps/s
        public static final double ROLLER_JERK = 4000.0; // Target jerk of 4000 rps/s/s
        public static final int ROLLER_KRAKEN_ID = 36;
    }
}

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
        public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // Normal full-stick translation cap in m/s; 50% of configured 12 V speed.
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.90).in(RadiansPerSecond); // Normal full-stick yaw cap in rad/s; 0.90 turns per second.
        public static final double DRIVE_DEADBAND = MAX_SPEED * 0.15; // Translation deadband in m/s; 15% of normal speed cap.
        public static final double ROTATIONAL_DEADBAND = MAX_ANGULAR_RATE * 0.15; // Yaw deadband in rad/s; 15% of normal yaw cap.

        public static final double SHOOT_LINEAR_SPEED_LIMIT_SCALE = 0.70; // Shooting fraction of normal speed: 0.70 = 70%, 1.0 = full speed.
        public static final double SHOOT_ANGULAR_SPEED_LIMIT_SCALE = 0.50; // Shooting fraction of normal yaw rate: 0.50 = 50%, 1.0 = full rate.
        public static final double SHOOT_MAX_ACCEL_MPS2 = 2.0; // Teleop shooting command ramp limit in m/s^2.
        public static final double SHOOT_MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = Units.degreesToRadians(180.0); // Teleop shooting yaw ramp limit in rad/s^2.

        public static final double SHORT_RANGE_ALIGN_DISTANCE_METERS = 0.75; // Use final pose alignment instead of pathfinding within this distance.
        public static final double SHORT_ALIGN_TRANSLATION_KP = 2.5; // Final X/Y alignment proportional gain.
        public static final double SHORT_ALIGN_TRANSLATION_KI = 0.0; // Final X/Y alignment integral gain.
        public static final double SHORT_ALIGN_TRANSLATION_KD = 0.0; // Final X/Y alignment derivative gain.
        public static final double SHORT_ALIGN_ROTATION_KP = 4.0; // Final heading alignment proportional gain.
        public static final double SHORT_ALIGN_ROTATION_KI = 0.0; // Final heading alignment integral gain.
        public static final double SHORT_ALIGN_ROTATION_KD = 0.0; // Final heading alignment derivative gain.
        public static final double SHORT_ALIGN_MAX_LINEAR_SPEED_MPS = 1.0; // Final alignment speed cap per X/Y axis in m/s.
        public static final double SHORT_ALIGN_MAX_ANGULAR_SPEED_RAD_PER_SEC = Units.degreesToRadians(180.0); // Heading speed cap in rad/s; also reused as profile acceleration.
        public static final double SHORT_ALIGN_POSITION_TOLERANCE_METERS = 0.05; // Allowed final distance from the alignment goal.
        public static final double SHORT_ALIGN_HEADING_TOLERANCE_DEG = 3.0; // Allowed final heading error in degrees.
        public static final double SHORT_ALIGN_SETTLE_TIME_SEC = 0.10; // Time both alignment tolerances must remain satisfied.

        public static final double AUTO_TRANSLATION_KP = 5.0; // PathPlanner translation proportional gain.
        public static final double AUTO_TRANSLATION_KI = 0.0; // PathPlanner translation integral gain.
        public static final double AUTO_TRANSLATION_KD = 0.0; // PathPlanner translation derivative gain.
        public static final double AUTO_ROTATION_KP = 5.0; // PathPlanner heading proportional gain.
        public static final double AUTO_ROTATION_KI = 0.0; // PathPlanner heading integral gain.
        public static final double AUTO_ROTATION_KD = 0.0; // PathPlanner heading derivative gain.
    }
    
    public static final class Turret {
        public static final int TURRET_KRAKEN_ID = 37; // Turret motor CAN ID on CANivore1.
        public static final double STALL_CURRENT_A = 6.0; // Minimum stator amps for the optional stall detector.
        public static final double STATOR_CURRENT_LIMIT_A = 50.0; // Motor torque/heating limit in amps.
        public static final double SUPPLY_CURRENT_LIMIT_A = 15.0; // Battery-current limit in amps.
        public static final double STALL_VEL_RPS = 0.25; // Maximum motor rot/s considered stalled; not a normal-move cutoff.
        public static final double STALL_TIME_S = 0.10; // Continuous low-speed/high-current dwell for stall detection.
        public static final double KS = 0.18; // Static-friction feedforward in volts.
        public static final double KV = 0.16; // Velocity feedforward in V/(motor rot/s).
        public static final double KA = 0.03; // Acceleration feedforward in V/(motor rot/s^2).
        public static final double KP = 10.0; // Position feedback gain in V/motor rotation.
        public static final double KI = 0.0; // Integral position gain in V/(motor rotation * s).
        public static final double KD = 0.88; // Derivative position gain in V/(motor rot/s).

        public static final double CRUISE_VELOCITY = 90.0; // Motion Magic cruise cap in motor rot/s; also used by the hood.
        public static final double ACCELERATION = 700.0; // Motion Magic acceleration cap in motor rot/s^2; also used by the hood.
        public static final double JERK = 12000.0; // Motion Magic jerk cap in motor rot/s^3; also used by the hood.
        public static final double MOTOR_ROTATIONS_PER_TURRET_ROTATION = 4; // Motor turns per turret turn; configured 100:25 reduction.
        public static final double POSITIVE_TURRET_YAW_MOTOR_SIGN = -1.0; // Motor direction corresponding to positive turret yaw.
        public static final Translation2d ROBOT_TO_TURRET_PIVOT = new Translation2d(-0.1651, -0.2159); // Pivot offset in robot-frame meters: +X forward, +Y left.
        public static final double FORWARD_REFERENCE_MOTOR_ROTATIONS = 0.00; // Encoder reading assigned when physically facing forward.
        public static final Rotation2d TURRET_ZERO_OFFSET = Rotation2d.fromDegrees(0.0); // Unused legacy yaw offset; current aim uses the forward reference.
        public static final Rotation2d PARK_ANGLE = Rotation2d.fromDegrees(0.0); // Logical turret angle used for parking.
        
        public static final double AIM_TOLERANCE_DEG = 4.0; // Final turret alignment error allowed for automatic feeding; loosened for moving-shot throughput.
        public static final double DIAGNOSTIC_TOLERANCE_DEG = 0.5; // Bench diagnostic tolerance, independent of feed alignment.
        public static final double MAX_VISION_AGE_FOR_SHOT_SEC = 0.25; // Accepted-vision age warning threshold; does not block feed.
        public static final double TRACKING_WINDOW_SEC = 1.0; // Rolling window for turret tracking-error metrics.
        public static final double STEP_ERROR_DEADBAND_DEG = 0.15; // Ignore smaller errors when counting oscillation sign changes.
        public static final double STEP_SETTLE_TIME_SEC = 0.10; // Continuous tolerance dwell before recording first settled time.

        public static final boolean TARGET_STABILIZATION_ENABLED = true; // Master switch for target hysteresis, revalidated shot reuse, and goal deadband; false restores raw aiming.
        public static final double TARGET_SWITCH_CONFIRM_SEC = 0.10; // A different target must remain selected this long before switching.
        public static final double PASSBACK_SWITCH_ADVANTAGE_METERS = 0.15; // New passback target must be this much nearer in Y before switching.
        public static final double PREVIOUS_SOLUTION_MAX_AGE_SEC = 0.10; // Maximum time since a fresh solution for reuse; every reused shot is fully revalidated.
        public static final boolean AIM_SMOOTHING_ENABLED = true; // Smooth field-relative aim corrections independently of target stabilization.
        public static final double AIM_SMOOTHING_TIME_CONSTANT_SEC = 0.060; // Larger = smoother but more tracking delay; zero bypasses smoothing.
        public static final double GOAL_DEADBAND_DEG = 0.15; // Ignore smaller accumulated goal changes; zero disables the deadband without delaying normal tracking.
        public static final double STABILIZATION_MAX_SAMPLE_GAP_SEC = 0.10; // Reset goal/target history after a larger sample gap or clock reversal.
        
        public static final double POSITION_TOLERANCE_ROT = AIM_TOLERANCE_DEG / 360.0; // Aim tolerance in turret turns; currently unused.

        // motor soft limits relative to the calibrated forward reference position
        public static final double TALON_SOFT_LIMIT_LOW = -0.74; // Lower motor-rotation bound relative to the forward reference.
        public static final double TALON_SOFT_LIMIT_HIGH = 4.019; // Upper motor-rotation bound relative to the forward reference.
        public static final double ABSOLUTE_TALON_SOFT_LIMIT_LOW = FORWARD_REFERENCE_MOTOR_ROTATIONS + TALON_SOFT_LIMIT_LOW; // Lower controller soft limit in encoder motor rotations.
        public static final double ABSOLUTE_TALON_SOFT_LIMIT_HIGH = FORWARD_REFERENCE_MOTOR_ROTATIONS + TALON_SOFT_LIMIT_HIGH; // Upper controller soft limit in encoder motor rotations.

        // Convert the lower raw bound into signed turret turns.
        private static final double LOGICAL_LIMIT_FROM_RAW_LOW =
            (TALON_SOFT_LIMIT_LOW * POSITIVE_TURRET_YAW_MOTOR_SIGN) / MOTOR_ROTATIONS_PER_TURRET_ROTATION;
        // Convert the upper raw bound into signed turret turns.
        private static final double LOGICAL_LIMIT_FROM_RAW_HIGH =
            (TALON_SOFT_LIMIT_HIGH * POSITIVE_TURRET_YAW_MOTOR_SIGN) / MOTOR_ROTATIONS_PER_TURRET_ROTATION;

        public static final double SOFT_LIMIT_LOW = Math.min(LOGICAL_LIMIT_FROM_RAW_LOW, LOGICAL_LIMIT_FROM_RAW_HIGH); // Lower legal logical turret position in turns.
        public static final double SOFT_LIMIT_HIGH = Math.max(LOGICAL_LIMIT_FROM_RAW_LOW, LOGICAL_LIMIT_FROM_RAW_HIGH); // Upper legal logical turret position in turns.
        
        public static final double SOFT_LIMIT_THRESHOLD = Rotation2d.fromDegrees(2.0).getRotations(); // Near-limit warning/feed margin in turret turns.
        
    }

    public static final class Vision {
        public static final AprilTagFieldLayout FIELD_LAYOUT_OBJECT = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded); // Loaded welded-field AprilTag geometry used by pose estimation.
        public static final boolean FORCE_ACCEPT_VISION_MEASUREMENTS = false; // Debug bypass of pose/ambiguity/age filters; timestamp sanity remains.

        // /* AprilTag Camera 1 */
        // public static final String APRILTAG_CAM1 = "APRILTAG_LEFT"; // PhotonVision name of the left camera.
        // // Left camera pose in robot coordinates; translation meters, rotation radians.
        // public static final Transform3d ROBOT_TO_CAM1 = new Transform3d(
        //     new Translation3d(
        //         Units.inchesToMeters(2.0),
        //         Units.inchesToMeters(13.0),
        //         Units.inchesToMeters(18.5)
        //     ),
        //     new Rotation3d(0.0, 0.0, Units.degreesToRadians(90.0))
        // );

        // /* AprilTag Camera 2 */
        // public static final String APRILTAG_CAM2 = "APRILTAG_RIGHT"; // PhotonVision name of the right camera.
        // // Right camera pose in robot coordinates; translation meters, rotation radians.
        // public static final Transform3d ROBOT_TO_CAM2 = new Transform3d(
        //     new Translation3d(
        //         Units.inchesToMeters(2.0),
        //         Units.inchesToMeters(-13.0),
        //         Units.inchesToMeters(17)
        //     ),
        //     new Rotation3d(0.0, 0.0, Units.degreesToRadians(-90.0))
        // );

        /* AprilTag Camera 1 */
        public static final String APRILTAG_CAM1 = "APRILTAG_LEFT";

        public static final Transform3d ROBOT_TO_CAM1 = new Transform3d(
            new Translation3d(
                -27.25 / 100.0,
                26.50 / 100.0,
                51.0 / 100.0
            ),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-10.0),  // Pitch: 10° upward
                Units.degreesToRadians(135.0)   // Yaw: rear-left
            )
        );

        /* AprilTag Camera 2 */
        public static final String APRILTAG_CAM2 = "APRILTAG_RIGHT";

        public static final Transform3d ROBOT_TO_CAM2 = new Transform3d(
            new Translation3d(
                -21.25 / 100.0,
                21.50 / 100.0,
                51.0 / 100.0
            ),
            new Rotation3d(
                0.0,
                Units.degreesToRadians(-10.0),  // Pitch: 10° upward
                Units.degreesToRadians(-45.0)   // Yaw: front-right
            )
        );

        /* Field layout, CA district is welded */
        public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded; // Welded-field layout selector; current loader uses the enum directly.

        public static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.0; // Maximum average camera-to-tag range for single-tag fallback.
        public static final double MAX_MULTI_TAG_DISTANCE_METERS = 2.0; // Maximum average range of tags used by a MultiTag estimate.
        public static final boolean USE_VISION_HEADING = false; // False fuses vision XY only; heading follows gyro/odometry and deliberate field resets.
        public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.15; // Largest accepted reported single-tag ambiguity.
        public static final double MAX_POSE_Z_METERS = 2.05; // Largest accepted absolute robot height from vision.
        public static final double MAX_POSE_ROLL_PITCH_DEG = 45.0; // Largest accepted absolute vision roll or pitch.
        public static final double MAX_MEASUREMENT_AGE_SEC = 0.6; // Reject captured vision older than this many seconds.
        public static final double VISION_HEALTHY_AGE_SEC = 0.70; // Accepted-vision age threshold for the health indicator.
        public static final double MULTI_TAG_BASE_XY_STD_DEV_METERS = 0.20; // Base translation uncertainty for multi-tag measurements.
        public static final double SINGLE_TAG_BASE_XY_STD_DEV_METERS = 0.50; // Base translation uncertainty for single-tag measurements.
        public static final double MULTI_TAG_BASE_THETA_STD_DEV_RAD = Units.degreesToRadians(3.0); // Base heading uncertainty for multi-tag measurements.
        public static final double SINGLE_TAG_BASE_THETA_STD_DEV_RAD = Units.degreesToRadians(8.0); // Base heading uncertainty for single-tag measurements.
        public static final double DISTANCE_STD_DEV_SCALE = 1.0 / 25.0; // Uncertainty multiplier is 1 + distance^2 * this factor (1/m^2).
    }

    public static final class FieldTargets {
        private static final double FIELD_LENGTH_METERS = Units.inchesToMeters(651.2); // Field length along +X from the blue-wall origin.
        private static final double FIELD_WIDTH_METERS = Units.inchesToMeters(317.7); // Field width along +Y.
        private static final double ALLIANCE_ZONE_DEPTH_METERS = Units.inchesToMeters(170); // Alliance-zone depth measured from each alliance wall.
        public static final double HUB_TARGET_HEIGHT_METERS = 1.83; // Modeled Hub target-plane height above the floor.
        public static final double BACK_TARGET_HEIGHT_METERS = 0.5; // Modeled passback target height above the floor.

        // 158.6 in is the alliance-zone boundary near the Hub face, not the Hub center.
        // Opposing centerline tags locate each Hub in the same field layout used by localization.
        public static final Translation2d BLUE_HUB_AIM_POINT =
            hubCenterBetweenTags(26, 20);
        public static final Translation2d RED_HUB_AIM_POINT =
            hubCenterBetweenTags(4, 10);
        public static final Translation2d BLUE_BACK_TARGET_UPPER = new Translation2d(0.48, 6.00); // Blue passback target on the higher-Y side, in meters.
        public static final Translation2d BLUE_BACK_TARGET_LOWER = new Translation2d(0.48, 2.50); // Blue passback target on the lower-Y side, in meters.
        public static final Translation2d RED_BACK_TARGET_UPPER = new Translation2d(16.1, 6.00); // Red passback target on the higher-Y side, in meters.
        public static final Translation2d RED_BACK_TARGET_LOWER = new Translation2d(16.1, 2.50); // Red passback target on the lower-Y side, in meters.
        public static final Translation2d OPPONENT_PASS_BACK_TARGET_UPPER = new Translation2d(8.295, 7.00); // Passback destination from the opponent zone, higher-Y side.
        public static final Translation2d OPPONENT_PASS_BACK_TARGET_LOWER = new Translation2d(8.295, 1.00); // Passback destination from the opponent zone, lower-Y side.

        // Blue zone polygon used to choose Hub versus passback aiming.
        public static final List<Translation2d> BLUE_ALLIANCE_ZONE = List.of(
            new Translation2d(0.0, 0.0),
            new Translation2d(ALLIANCE_ZONE_DEPTH_METERS, 0.0),
            new Translation2d(ALLIANCE_ZONE_DEPTH_METERS, FIELD_WIDTH_METERS),
            new Translation2d(0.0, FIELD_WIDTH_METERS)
        );

        // Red zone polygon used to choose Hub versus passback aiming.
        public static final List<Translation2d> RED_ALLIANCE_ZONE = List.of(
            new Translation2d(FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS, 0.0),
            new Translation2d(FIELD_LENGTH_METERS, 0.0),
            new Translation2d(FIELD_LENGTH_METERS, FIELD_WIDTH_METERS),
            new Translation2d(FIELD_LENGTH_METERS - ALLIANCE_ZONE_DEPTH_METERS, FIELD_WIDTH_METERS)
        );

        private FieldTargets() {}

        private static Translation2d hubCenterBetweenTags(int firstId, int oppositeId) {
            Translation2d first = Vision.FIELD_LAYOUT_OBJECT.getTagPose(firstId).orElseThrow()
                .toPose2d().getTranslation();
            Translation2d opposite = Vision.FIELD_LAYOUT_OBJECT.getTagPose(oppositeId).orElseThrow()
                .toPose2d().getTranslation();
            return first.plus(opposite).div(2.0);
        }

        public static Translation2d getHubForAlliance(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RED_HUB_AIM_POINT : BLUE_HUB_AIM_POINT;
        }

        public static Translation3d getHubAimPoint3d(DriverStation.Alliance alliance) {
            Translation2d hubPoint = getHubForAlliance(alliance);
            return new Translation3d(hubPoint.getX(), hubPoint.getY(), HUB_TARGET_HEIGHT_METERS);
        }

        public static List<Translation2d> getBackTargetsForAlliance(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red
                ? List.of(RED_BACK_TARGET_UPPER, RED_BACK_TARGET_LOWER)
                : List.of(BLUE_BACK_TARGET_UPPER, BLUE_BACK_TARGET_LOWER);
        }

        public static Translation2d getPreferredBackTargetForAlliance(
            DriverStation.Alliance alliance,
            double robotPoseY
        ) {
            Translation2d bestTarget = null;
            double bestDistance = Double.POSITIVE_INFINITY;

            for (Translation2d candidate : getBackTargetsForAlliance(alliance)) {
                double distance = Math.abs(candidate.getY() - robotPoseY);
                if (distance < bestDistance
                    || (Math.abs(distance - bestDistance) <= 1e-9
                        && (bestTarget == null || candidate.getY() < bestTarget.getY()))) {
                    bestTarget = candidate;
                    bestDistance = distance;
                }
            }

            return bestTarget;
        }

        public static Translation3d getPreferredBackTarget3dForAlliance(
            DriverStation.Alliance alliance,
            double robotPoseY
        ) {
            Translation2d backTarget = getPreferredBackTargetForAlliance(alliance, robotPoseY);
            return new Translation3d(backTarget.getX(), backTarget.getY(), BACK_TARGET_HEIGHT_METERS);
        }

        public static Translation2d getPreferredOpponentPassBackTarget(double robotPoseY) {
            Translation2d bestTarget = null;
            double bestDistance = Double.POSITIVE_INFINITY;

            for (Translation2d candidate : List.of(OPPONENT_PASS_BACK_TARGET_UPPER, OPPONENT_PASS_BACK_TARGET_LOWER)) {
                double distance = Math.abs(candidate.getY() - robotPoseY);
                if (distance < bestDistance
                    || (Math.abs(distance - bestDistance) <= 1e-9
                        && (bestTarget == null || candidate.getY() < bestTarget.getY()))) {
                    bestTarget = candidate;
                    bestDistance = distance;
                }
            }

            return bestTarget;
        }

        public static Translation3d getPreferredOpponentPassBackTarget3d(double robotPoseY) {
            Translation2d backTarget = getPreferredOpponentPassBackTarget(robotPoseY);
            return new Translation3d(backTarget.getX(), backTarget.getY(), BACK_TARGET_HEIGHT_METERS);
        }

        public static List<Translation2d> getAllianceZone(DriverStation.Alliance alliance) {
            return alliance == DriverStation.Alliance.Red ? RED_ALLIANCE_ZONE : BLUE_ALLIANCE_ZONE;
        }
    }

    public static final class Intake {
        public static final int ROLLER_KRAKEN_ID = 40; // Intake roller CAN ID on CANivore1.
        public static final int EXTENSION_KRAKEN_MASTER_ID = 41; // Pivot master CAN ID on CANivore1.
        public static final int EXTENSION_KRAKEN_FOLLOWER_ID = 42; // Pivot follower CAN ID; configured opposed to the master.
        public static final double EXTENSION_EXTEND_VELOCITY = 50.0; // Deploy cruise cap in motor rot/s.
        public static final double EXTENSION_EXTEND_ACCELERATION = 100.0; // Deploy acceleration cap in motor rot/s^2.
        public static final double EXTENSION_EXTEND_JERK = 500.0; // Deploy jerk cap in motor rot/s^3; zero removes S-curve smoothing.
        public static final double EXTENSION_COLLAPSE_VELOCITY = 90.0; // Retract cruise cap in motor rot/s.
        public static final double EXTENSION_COLLAPSE_ACCELERATION = 400.0; // Retract acceleration cap in motor rot/s^2.
        public static final double EXTENSION_COLLAPSE_JERK = 4000.0; // Retract jerk cap in motor rot/s^3; zero removes S-curve smoothing.
        public static final double EXTENSION_STATOR_CURRENT_LIMIT_A = 100.0; // Torque/heating limit in amps per pivot motor.
        public static final double EXTENSION_SUPPLY_CURRENT_LIMIT_A = 30.0; // Battery-current limit in amps per pivot motor.
        public static final double ROLLER_STATOR_CURRENT_LIMIT_A = 30.0; // Intake roller torque/heating limit in amps.
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_A = 20.0; // Intake roller battery-current limit in amps.
        public static final double INTAKE_STALL_CURRENT = 6.0; // Minimum master stator amps for homing/anti-jam stall detection.
        public static final double INTAKE_STALL_VELOCITY = 10; // Maximum master motor rot/s considered stalled.
        public static final double STALL_TIME_S = 0.25; // Continuous stall-condition dwell in seconds.
        public static final double HOMING_TIMEOUT_SEC = 3.0; // Provisional homing deadline; validate travel time on hardware.
        public static final double HOMING_VOLTAGE = -1.0; // Open-loop master volts toward the collapsed homing stop.

        public static final double INTAKE_TARGET_ROT_TOLERANCE = 0.02; // Allowed pivot position error in master motor rotations.

        public static final double INTAKE_COLLAPSED_POS = 0.0; // Collapsed master motor position; startup zero assumes intake is in.
        public static final double INTAKE_EXTENDED_POS = 1.500; // Deployed master motor position, relative to collapsed zero.
        // Master motor position reached during an optional retract pulse.
        public static final double ANTI_JAM_PULSE_RETRACT_POS =
            INTAKE_COLLAPSED_POS;
        public static final int ANTI_JAM_PULSE_CYCLES = 0; // Retract/deploy cycles per pulse; zero disables automatic pulsing.
        public static final double ANTI_JAM_PULSE_STEP_TIMEOUT_SEC = 0.25; // Maximum wait for each anti-jam position step.
        public static final double ANTI_JAM_PULSE_DWELL_SEC = 0.05; // Pause after each anti-jam position step.

        public static final double ROLLER_VELOCITY = 42.0; // Forward intake roller target in motor rot/s.
        public static final double ROLLER_REVERSE_VELOCITY = -21.0; // Intake roller motor rot/s while left trigger is held.
    }

    public static final class Shooter {
        public static final int HOOD_KRAKEN_ID = 21; // Hood motor CAN ID on CANivore1.
        public static final int ROLLER_KRAKEN_ID = 31; // Flywheel motor CAN ID on CANivore1.
        public static final double HOOD_STATOR_CURRENT_LIMIT_A = 40.0; // Hood torque/heating limit in amps.
        public static final double HOOD_SUPPLY_CURRENT_LIMIT_A = 25.0; // Hood battery-current limit in amps.
        public static final double ROLLER_STATOR_CURRENT_LIMIT_A = 70.0; // Flywheel torque/heating limit in amps.
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_A = 35.0; // Flywheel battery-current limit in amps.

        public static final double KS = 0.25; // Static-friction feedforward in volts.
        public static final double KV = 0.12; // Velocity feedforward in V/(motor rot/s).
        public static final double KA = 0.01; // Acceleration feedforward in V/(motor rot/s^2).
        public static final double KP = 35.0; // Position feedback gain in V/motor rotation.
        public static final double KI = 0.00; // Integral position gain in V/(motor rotation * s).
        public static final double KD = 0.20; // Derivative position gain in V/(motor rot/s).

        public static final double ROLLER_KS = 0.25; // Static-friction feedforward in volts.
        public static final double ROLLER_KV = 0.12; // Velocity feedforward in V/(motor rot/s).
        public static final double ROLLER_KA = 0.01; // Acceleration feedforward in V/(motor rot/s^2).
        public static final double ROLLER_KP = 0.20; // Velocity feedback gain in V/(motor rot/s).
        public static final double ROLLER_KI = 0.00; // Integral velocity gain in V/motor rotation.
        public static final double ROLLER_KD = 0.00; // Derivative velocity gain in V/(motor rot/s^2).
        public static final double ROLLER_DIRECTION_SIGN = -1.0; // Sign converting positive exit speed into motor velocity.
        public static final double ROLLER_VELOCITY = -31.98; // Default flywheel target in motor rot/s; auto aim updates it.
        public static final double ROLLER_ACCELERATION = 400.0; // Flywheel acceleration cap in motor rot/s^2.
        public static final double ROLLER_JERK = 4000.0; // Flywheel jerk cap in motor rot/s^3; zero removes S-curve smoothing.

        public static final double ROLLER_START_TOLERANCE_RPS = 3.0; // Allowed entry underspeed in motor rot/s; combined with the fraction below.
        public static final double ROLLER_START_TOLERANCE_SCALE = 0.10; // Fractional speed-error entry band; 0.10 = 10% of target magnitude.
        public static final double ROLLER_RUNNING_TOLERANCE_RPS = 5.0; // Absolute running band in motor rot/s; combined with the fraction below.
        public static final double ROLLER_RUNNING_TOLERANCE_SCALE = 0.15; // Allowed running underspeed; 0.15 = 15% of target magnitude. Overspeed does not block.
        public static final double HOOD_POSITION_TOLERANCE_ROT = 0.08; // Feed alignment tolerance in motor turns; about 0.93 launch degrees.
        public static final double MAX_MECHANISM_FEEDBACK_AGE_SEC = 0.10; // Maximum hood/flywheel sample age accepted for feed readiness.
        public static final double EXIT_HEIGHT_METERS = 0.4064; // Modeled Fuel release height above the floor.
        public static final double HOOD_POSITION_PER_LAUNCH_DEG_SLOPE = 0.0857913; // Calibration slope: hood motor rotations per launch degree.
        public static final double HOOD_POSITION_PER_LAUNCH_DEG_INTERCEPT = -7.49442; // Calibration offset: hood motor rotations at zero launch degrees.
        public static final double MIN_HOOD_POSITION_ROT = -2.346; // Lower modeled hood-command bound; not a configured hardware soft limit.
        public static final double MAX_HOOD_POSITION_ROT = -0.631116; // Upper modeled hood bound; also used by the trench command.
        public static final double FAR_SHOT_DISTANCE_METERS = 4.0; // Distance where the legacy solver applies its far-shot hood floor.
        public static final double FAR_SHOT_MIN_HOOD_POSITION_ROT = -1.40; // Legacy far-Hub hood floor in motor rotations.
        public static final double PASS_BACK_FAR_SHOT_MIN_HOOD_POSITION_ROT = MIN_HOOD_POSITION_ROT; // Legacy far-passback hood floor in motor rotations.
        public static final double MAX_ROLLER_RPS = 100.0; // Maximum allowed flywheel target magnitude in motor rot/s.
        public static final double MAX_EXIT_SPEED_MPS = 11.120291; // Largest modeled exit speed allowed by shot requests.
        public static final double ROLLER_RPS_PER_EXIT_SPEED_SLOPE = 7.763381; // Calibration slope: motor rot/s per m/s of Fuel exit speed.
        public static final double ROLLER_RPS_PER_EXIT_SPEED_INTERCEPT = 13.046605; // Calibration speed offset before applying the motor direction sign.
    }

    /** Manual motor-speed trials; values are motor RPS, not RPM or Fuel exit speed. */
    public static final class LaunchSpeedCalibration {
        public static final boolean USE_DASHBOARD_RPS = true;
        public static final double ROLLER_SPEED_RPS = 25.0;
        public static final double LAUNCH_ANGLE_DEG = 70.0;
    }

    public static final class ShotMap {
        public static final boolean USE_ROBUST_SOTM = true; // Select the release-prediction and final-trajectory evaluator.
        public static final boolean USE_LEGACY_SOLVER_ROLLBACK = false; // Explicit legacy mode; rejected robust shots never silently fall back.
        public static final boolean TILT_COMPENSATION_ENABLED = false; // Apply measured pitch/roll to shooting; false assumes level while preserving yaw/motion prediction.
        public static final double ROLLER_RPS_OUTPUT_SCALE = 1.05; // Final auto-aim motor RPS multiplier; 1.0 disables trim. Manual calibration is unscaled.
        public static final boolean FAR_HUB_RPS_TRIM_ENABLED = true; // Empirical correction for the observed near-lip miss; disable for an untrimmed far-shot comparison.
        public static final double FAR_HUB_RPS_TRIM_MULTIPLIER = 1.04; // Additional 1.2% after global trim: -86.95 becomes -87.9934 RPS at full distance.
        public static final double FAR_HUB_RPS_TRIM_START_METERS = 4.9; // Start smoothly adding the correction at this predicted release-to-Hub distance.
        public static final double FAR_HUB_RPS_TRIM_FULL_METERS = 5.5; // Full correction here and farther; hood settings remain independent.
        public static final int FAR_HUB_SHOT_MAP_TRIAL = 1; // 1 = generated 63-degree trial, 2 = 61-degree trial, 0 = original map plus optional runtime flattening below.
        public static final boolean FAR_HUB_FLAT_SHOT_ENABLED = true; // Recompute far-Hub shots with a flatter field launch angle; false restores the map.
        public static final double FAR_HUB_FLAT_START_METERS = 4.9; // Start smoothly flattening at this release-to-Hub distance, not a field-X boundary.
        public static final double FAR_HUB_FLAT_FULL_METERS = 5.5; // Reach the requested far angle here; must exceed START_METERS.
        public static final double FAR_HUB_FLAT_ANGLE_DEG = 63.0; // Far field launch pitch; RPS is solved together with angle and robot motion.
        public static final double BASE_RELEASE_LATENCY_SEC = 0.14; // Provisional prediction horizon to Fuel departure; measure on hardware.
        public static final double ACCEL_FILTER_ALPHA = 0.25; // Weight of the new acceleration sample; larger values filter less.
        public static final double MAX_ACCEL_SAMPLE_DT_SEC = 0.10; // Reject acceleration derivatives across larger odometry time gaps.
        public static final double MAX_FIELD_ACCEL_MPS2 = 8.0; // Prediction clamp per field acceleration axis; feed still checks raw acceleration.
        public static final double MAX_ANGULAR_ACCEL_RAD_PER_SEC2 = Units.degreesToRadians(900.0); // Prediction angular-acceleration clamp; feed still checks raw acceleration.
        public static final double MIN_TRANSLATION_SCALE_WHEN_NOT_READY = 0.65; // Unused advisory scale; current drive limits use Drivebase settings.
        public static final double MIN_ROTATION_SCALE_WHEN_NOT_READY = 0.55; // Unused advisory scale; current yaw limits use Drivebase settings.
        public static final boolean FEED_INTERLOCK_ENABLED = true; // Master feed-readiness switch: true gates normal/autonomous/calibration feed; false bypasses shot readiness. Motor guards and calibration input validation remain.
        public static final boolean FEED_REQUIRE_TURRET_ALIGNMENT = false; // Require turret angular tolerance before feeding; false allows feeding while tracking. Travel limits still apply.
        public static final boolean CONTINUE_FEED_AFTER_INITIAL_READY = true; // After automatic feed starts, ignore ShotReady fluctuations until release or a hard actuator/operation stop. False restores continuous gating.
        public static final double FEED_RECOVERY_READY_TIME_SEC = 0.05; // Continuous readiness dwell after an automatic feed interruption.
        public static final double OUTTAKE_DELAY_AFTER_READY_SEC = 0.05; // Delay from readiness to final-feed start; force feed skips it.
        public static final double INDEXER_DELAY_AFTER_OUTTAKE_SEC = 0.025; // Extra delay before spindexer start; force feed skips it.
        public static final double MAX_SOLUTION_AGE_SEC = 0.10; // Maximum aim-evaluation age allowed for automatic feeding.
        public static final double SHOOT_MEASURED_SPEED_HEADROOM_SCALE = 1.20; // Measured speed allowance above shooting command caps; 1.20 = +20%.
        public static final double SHOOT_MEASURED_ACCEL_HEADROOM_SCALE = 1.50; // Measured acceleration allowance above command ramps; 1.50 = +50%.
        // Feed speed cap for both current and predicted release motion.
        public static final double MAX_SHOOT_SPEED_MPS = Drivebase.MAX_SPEED
            * Drivebase.SHOOT_LINEAR_SPEED_LIMIT_SCALE * SHOOT_MEASURED_SPEED_HEADROOM_SCALE;
        // Feed acceleration cap applied to raw and filtered vector magnitudes.
        public static final double MAX_SHOOT_ACCEL_MPS2 = Drivebase.SHOOT_MAX_ACCEL_MPS2
            * SHOOT_MEASURED_ACCEL_HEADROOM_SCALE;
        // Feed yaw-rate cap for current and predicted release motion.
        public static final double MAX_SHOOT_OMEGA_RAD_PER_SEC = Drivebase.MAX_ANGULAR_RATE
            * Drivebase.SHOOT_ANGULAR_SPEED_LIMIT_SCALE * SHOOT_MEASURED_SPEED_HEADROOM_SCALE;
        // Feed yaw-acceleration cap applied to raw and filtered magnitudes.
        public static final double MAX_SHOOT_ALPHA_RAD_PER_SEC2 = Drivebase.SHOOT_MAX_ANGULAR_ACCEL_RAD_PER_SEC2
            * SHOOT_MEASURED_ACCEL_HEADROOM_SCALE;
        public static final double MAX_MOTION_AGE_SEC = 0.05; // Maximum measured-motion age allowed for automatic aiming/feeding.
    }

    public static final class PowerLogging {
        public static final int REV_PDH_CAN_ID = 1; // PDH CAN ID to verify; a negative value uses WPILib discovery.
        public static final double MOTOR_ELECTRICAL_RATE_HZ = 50.0; // Requested current/voltage CAN update rate; recorder runs each robot loop.
        public static final double TURRET_MOTION_RATE_HZ = 100.0; // Requested turret position/velocity and motion-logger rate.
        public static final double SLOW_HEALTH_RATE_HZ = 10.0; // Temperature, fault, reset and configuration recording rate.
        public static final double DASHBOARD_RATE_HZ = 5.0; // Power logger dashboard refresh rate.
        public static final int CONFIG_APPLY_ATTEMPTS = 3; // Maximum CTRE configuration apply attempts before reporting failure.
        public static final String BATTERY_LABEL = "UNKNOWN"; // Recorded battery identity; replace UNKNOWN for identified test runs.
        public static final String CONFIGURATION_ID = "ultimate-sotm-v3-throughput"; // Recorded software-settings label; update when identifying a new configuration.

        private PowerLogging() {}
    }

    public static final class Outtake {
        public static final double ROLLER_STATOR_CURRENT_LIMIT_A = 40.0; // Final-feed motor torque/heating limit in amps.
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_A = 25.0; // Final-feed motor battery-current limit in amps.
        public static final double ROLLER_KS = 0.25; // Static-friction feedforward in volts.
        public static final double ROLLER_KV = 0.12; // Velocity feedforward in V/(motor rot/s).
        public static final double ROLLER_KA = 0.01; // Acceleration feedforward in V/(motor rot/s^2).
        public static final double ROLLER_KP = 0.11; // Velocity feedback gain in V/(motor rot/s).
        public static final double ROLLER_KI = 0.00; // Integral velocity gain in V/motor rotation.
        public static final double ROLLER_KD = 0.00; // Derivative velocity gain in V/(motor rot/s^2).
        public static final double ROLLER_VELOCITY = -35; // Final-feed target in motor rot/s; negative is the feeding direction.
        public static final double ROLLER_ACCELERATION = 400.0; // Final-feed acceleration cap in motor rot/s^2.
        public static final double ROLLER_JERK = 4000.0; // Final-feed jerk cap in motor rot/s^3; zero removes S-curve smoothing.
        public static final int ROLLER_KRAKEN_ID = 25; // Final-feed motor CAN ID on CANivore1.
    }
    public static final class Indexer {
        public static final double ROLLER_STATOR_CURRENT_LIMIT_A = 50.0; // Spindexer motor torque/heating limit in amps.
        public static final double ROLLER_SUPPLY_CURRENT_LIMIT_A = 40.0; // Spindexer motor battery-current limit in amps.
        public static final double ROLLER_KS = 0.25; // Static-friction feedforward in volts.
        public static final double ROLLER_KV = 0.12; // Velocity feedforward in V/(motor rot/s).
        public static final double ROLLER_KA = 0.01; // Acceleration feedforward in V/(motor rot/s^2).
        public static final double ROLLER_KP = 1.0; // Velocity gain in V/(motor rot/s); provisional aggressive bench setting.
        public static final double ROLLER_KI = 0.00; // Integral velocity gain in V/motor rotation.
        public static final double ROLLER_KD = 0.00; // Derivative velocity gain in V/(motor rot/s^2).
        public static final double ROLLER_VELOCITY = -100.0; // Target motor rot/s; actual spinner speed depends on the unverified pulley ratio.
        public static final double ROLLER_ACCELERATION = 800.0; // Spindexer acceleration cap in motor rot/s^2.
        public static final double ROLLER_JERK = 0.0; // Motor rot/s^3; zero disables S-curve smoothing.
        public static final int ROLLER_KRAKEN_ID = 36; // Spindexer motor CAN ID on CANivore1.
    }
}

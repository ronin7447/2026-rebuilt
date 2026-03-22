package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
    /* Drivebase Constants */
    public static final class Drivebase {
        public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
        public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
        public static final double DRIVE_DEADBAND = MAX_SPEED * 0.2; // 20% of max speed as deadband
        public static final double ROTATIONAL_DEADBAND = MAX_ANGULAR_RATE * 0.2; // 20% of max angular rate as deadband
    }
    
    public static final class Turret {
        public static final int TURRET_KRAKEN_ID = 12; // Turret X44 Kraken ID
        public static final double STALL_CURRENT_A = 6.0;
        public static final double STALL_VEL_RPS = 0.25;
        public static final double STALL_TIME_S = 0.10;
        public static final double KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double KP = 35.0; // 35 V per rotation of position error
        public static final double KI = 0.00; // no output for integrated error
        public static final double KD = 0.20; // 0.20 V output for error derivative

        public static final double CRUISE_VELOCITY = 6.0;
        public static final double ACCELERATION = 5.0;
        public static final double JERK = 7.0;
        public static final double POSITION_TOLERANCE_ROT = 0.10;
        public static final double SOFT_LIMIT_LOW = Double.NEGATIVE_INFINITY; // TODO: replace with real turret min rotation
        public static final double SOFT_LIMIT_HIGH = Double.POSITIVE_INFINITY; // TODO: replace with real turret max rotation
        public static final double SOFT_LIMIT_THRESHOLD = 0.0; // TODO: replace after real soft limits are known
        
        
        

    }

    public static final class Vision {

        /* AprilTag Camera 1 */
        public static final String APRILTAG_CAM1 = "APRILCAM1";
        public static final Transform3d ROBOT_TO_CAM1 = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0.0, Math.toRadians(-15), 0.0) // outward 15 deg
        );

        /* AprilTag Camera 2 */
        public static final String APRILTAG_CAM2 = "APRILCAM2";
        public static final Transform3d ROBOT_TO_CAM2 = new Transform3d(
            new Translation3d(0, 0, 0),
            new Rotation3d(0.0, Math.toRadians(-15), 0.0) // outward 15 deg
        );

        /* Field layout, CA district is welded */
        public static final AprilTagFields FIELD_LAYOUT = AprilTagFields.k2026RebuiltWelded;

        public static final double MAX_SINGLE_TAG_AMBIGUITY = 0.20; // throw the estimation away if the tag ambiguity > 0.20
        public static final double MAX_POSE_Z_METERS = 0.75; // estimated robot "z" higher than 0.75m is probably bad estimation
        public static final double MAX_CORRECTION_METERS = 1.5; // compare the existing pose estimate and vision pose, reject the vision pose if the jump is bigger than 1.5m
    }

    public static final class Intake {
        public static final int ROLLER_KRAKEN_ID = 12;
        public static final int EXTENSION_KRAKEN_MASTER_ID = 13;
        public static final int EXTENSION_KRAKEN_FOLLOWER_ID = 14;
        public static final double EXTENSION_VELOCITY = 15.0;
        public static final double EXTENSION_ACCELERATION = 30.0;
        public static final double EXTENSION_JERK = 60.0;


        public static final double INTAKE_TARGET_ROT_TOLERANCE = 0.02;

        public static final double INTAKE_COLLAPSED_POS = 0.36;
        public static final double INTAKE_EXTENDED_POS = 2.420;

        public static final double ROLLER_VELOCITY = 72.0;
    }

    public static final class Shooter {
        public static final int HOOD_KRAKEN_ID = 23;
        public static final int ROLLER_KRAKEN_ID = 24;

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
        public static final double ROLLER_VELOCITY = 72.0;
        public static final double ROLLER_ACCELERATION = 400.0; // Target acceleration of 400 rps/s
        public static final double ROLLER_JERK = 4000.0; // Target jerk of 4000 rps/s/s

    }
}

package frc.robot.constants;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
        public static final double STALL_CURRENT_A = 70.0; 
        public static final double STALL_VEL_RPS = 0.25;
        public static final double STALL_TIME_S = 0.25;
        public static final double KS = 0.25; // Add 0.25 V output to overcome static friction
        public static final double KV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        public static final double KA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        public static final double KP = 35.0; // 35 V per rotation of position error
        public static final double KI = 0.00; // no output for integrated error
        public static final double KD = 0.20; // 0.20 V output for error derivative

        public static final double CRUISE_VELOCITY = 10.0; // 10 rps
        public static final double ACCELERATION = 40.0; // 40 rps /s
        public static final double JERK = 400.0; // Target jerk of 400 rps/s/s 
        

    }
}
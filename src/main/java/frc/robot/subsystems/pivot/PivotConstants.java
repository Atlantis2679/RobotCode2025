package frc.robot.subsystems.pivot;

public class PivotConstants {
    public static final int PIVOT_MAX_VOLTAGE = 12;
    public static final int PIVOT_CURRENT_LIMIT = 30;

    public static final double ANGLE_OFFSET = 256;
    public static final double UPPER_BOUND = 248;
    public static final double LOWER_BOUND = 150;
    
    public static final double MAX_VELOCITY_DEG_PER_SEC = 0;
    public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 0;
    public static final double MAX_ANGLE_DEGREES = 0; // was 120
    public static final double MIN_ANGLE_DEGREES = 0;
    public static final double ANGLE_TOLERENCE_DEGREES = 0;
    public static final double KP = 0;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0;
    public static final double KA = 0;
    public static final double KV = 0;
    public static final double KG = 0;

    public static final double ENCODER_CONNECTED_DEBAUNCER_SEC = 0;

    public static class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KG = 0;

        public static final double JKG_METERS_SQUARED = 0.04;
        public static final double JOINT_GEAR_RATIO = 60;
        public static final double TURNING_MIN_DEGREES_DEG = 115;
        public static final double TURNING_MAX_DEGREES_DEG = 110;    

        public static final double ARM_LENGTH_M = 0.42;
    }
}
package frc.robot.subsystems.pivot;

public class PivotConstants {
    public static final int PIVOT_MAX_VOLTAGE = 12;
    public static final int PIVOT_CURRENT_LIMIT = 30;

    public static final double INITIAL_OFFSET = 116;
    public static final double FULL_ROTATION = 360;
    public static final double UPPER_BOUND = 140;
    
    public static final double MAX_VELOCITY_DEG_PER_SEC = 70;
    public static final double MAX_ACCELERATION = 70;
    public static final double MAX_ANGLE_DEGREES = 130;
    public static final double MIN_ANGLE_DEGREES = -115;
    public static final double MESURED_ANGLE_TOLERENCE_DEGREES = 1.5;

    public static final double KP = 0.01;
    public static final double KI = 0;
    public static final double KD = 0;

    public static final double KS = 0;
    public static final double KA = 0;
    public static final double KV = 0;
    public static final double KG = 0.015;

    public static class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0;
        public static final double SIM_KG = 0;

        public static final double JKG_METERS_SQUARED = 0;
        public static final double JOINT_GEAR_RATIO = 0;
        public static final double TURNING_MIN_DEGREES = 0;
        public static final double TURNING_MAX_DEGREES = 0.25;    

        public static final double ARM_LENGTH = 0;
    }
}

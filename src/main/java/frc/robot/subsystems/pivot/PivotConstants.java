package frc.robot.subsystems.pivot;

public class PivotConstants {
    public static final int PIVOT_MAX_VOLTAGE = 12;
    public static final int PIVOT_CURRENT_LIMIT = 30;

    public static final double ANGLE_OFFSET = -12;
    public static final double UPPER_BOUND = 205;
    public static final double LOWER_BOUND = -155;
    
    public static final double MAX_VELOCITY_DEG_PER_SEC = 250;
    public static final double MAX_ACCELERATION = 1000;
    public static final double MAX_ANGLE_DEGREES = 95; // was 120
    public static final double MIN_ANGLE_DEGREES = -110;
    public static final double ANGLE_TOLERENCE_DEGREES = 4;

    public static final double KP = 0.07;
    public static final double KI = 0.07;
    public static final double KD = 0.007;

    public static final double KS = 0;
    public static final double KA = 0;
    public static final double KV = 0.048;
    public static final double KG = 0.18;

    public static final double ENCODER_CONNECTED_DEBAUNCER_SEC = 0.1;


    public static class Sim {
        public static final double SIM_KS = 0;
        public static final double SIM_KA = 0;
        public static final double SIM_KV = 0.0005;
        public static final double SIM_KG = 0.03;

        public static final double JKG_METERS_SQUARED = 0.004;
        public static final double JOINT_GEAR_RATIO = 1/60.0;
        public static final double TURNING_MIN_DEGREES = 115;
        public static final double TURNING_MAX_DEGREES = 110;    

        public static final double ARM_LENGTH = 50;
    }
}
package frc.robot.allcommands;

public class AllCommandsConstants {
    public static final double FUNNEL_INTAKE_SPEED = 0.5;
    public static final double FUNNEL_PASSING_SPEED = 0.5;

    public static final double GRIPPER_BACK_LOADING_VOLTAGE = 7.5;
    public static final double GRIPPER_LEFT_LOADING_VOLTAGE = 0.15;
    public static final double GRIPPER_RIGHT_LOADING_VOLTAGE = 0.15;
    public static final double GRIPPER_BACK_L1_VOLTAGE = 5;
    public static final double GRIPPER_RIGHT_L1_VOLTAGE = 5.2;
    public static final double GRIPPER_LEFT_L1_VOLTAGE = 2.15;
    // public static final double GRIPPER_L2_VOLTAGE = 7;
    public static final double GRIPPER_OUTTAKE_L3_VOLTAGE = -10;
    public static final double GRIPPER_BACK_L3_VOLTAGE = -5;

    public static final double PIVOT_ANGLE_FOR_INTAKE = -105;
    public static final double PIVOT_ANGLE_FOR_L1 = -80;
    public static final double PIVOT_ANGLE_FOR_L2 = 17;//25
    public static final double PIVOT_ANGLE_FOR_L3 = 80;
    public static final double PIVOT_ANGLE_FOR_REST = -90;
    
    public static final double PIVOT_TUNEABLE_ANGLE = 0;
    public static final double GRIPPER_BACK_TUNEABLE_VOLTAGE = 0; 
    public static final double GRIPPER_RIGHT_TUNEABLE_VOLTAGE = 0;
    public static final double GRIPPER_LEFT_TUNEABLE_VOLTAGE = 0;

    public static final long TEST_WIZARD_DELAY = 1;

    public final static int SECONDS_FOR_LEDS_DEFAULT = 1;

    public final static double AUTO_DRIVE_SECONDS = 1.5;
    public final static double AUTO_DRIVE_VOLTAGE_PERCANTAGE = 0.2;

    public final static double MAX_DISTANCE_FOR_GET_TO_POSE = 1;

    public static class ManualControllers {
        public static final double FUNNEL_SPEED_MULTIPLAYER = 0.5;
        public static final double GRIPPER_LEFT_SPEED_MULTIPLAYER = 0.5;
        public static final double GRIPPER_RIGHT_SPEED_MULTIPLAYER = 0.5;
        public static final double GRIPPER_BACK_SPEED_MULTIPLAYER = 0.5;
        public static final double PIVOT_SPEED_MULTIPLAYER = 0.05;
    }
}

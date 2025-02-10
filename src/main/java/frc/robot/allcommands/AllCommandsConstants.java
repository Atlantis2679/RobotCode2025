package frc.robot.allcommands;

public class AllCommandsConstants {
    public static final double FUNNEL_INTAKE_SPEED = 0.2;
    public static final double FUNNEL_PASSING_SPEED = 0.35;

    public static final double GRIPPER_INTAKE_VOLTAGE = 6;
    public static final double GRIPPER_LOADING_VOLTAGE = 4;
    public static final double GRIPPER_RIGHT_L1_VOLTAGE = 7;
    public static final double GRIPPER_LEFT_L1_VOLTAGE = 5;
    // public static final double GRIPPER_L2_VOLTAGE = 7;
    public static final double GRIPPER_L3_VOLTAGE = -12;


    public static final double PIVOT_ANGLE_FOR_INTAKE = -111;
    public static final double PIVOT_ANGLE_FOR_L1 = -80;
    public static final double PIVOT_ANGLE_FOR_L2 = 25;
    public static final double PIVOT_ANGLE_FOR_L3 = 82;
    
    public static final double PIVOT_TUNEABLE_ANGLE = 0;
    public static final double GRIPPER_RIGHT_TUNEABLE_VOLTAGE = 0;
    public static final double GRIPPER_LEFT_TUNEABLE_VOLTAGE = 0; 

    public static class ManualControllers {
        public static final double FUNNEL_SPEED_MULTIPLAYER = 1;
        public static final double GRIPPER_LEFT_SPEED_MULTIPLAYER = 1;
        public static final double GRIPPER_RIGHT_SPEED_MULTIPLAYER = 1;
        public static final double PIVOT_SPEED_MULTIPLAYER = 0.008;
    }
}

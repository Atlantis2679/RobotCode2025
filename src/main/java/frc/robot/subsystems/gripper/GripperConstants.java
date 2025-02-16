package frc.robot.subsystems.gripper;

public class GripperConstants {
    public static final int LEFT_MOTOR_CURRENT_LIMIT = 30;
    public static final int RIGHT_MOTOR_CURRENT_LIMIT = 20;

    /* Gripper: */
    public static final double LEFT_MOTOR_MAX_VOLTAGE = 12;
    public static final double RIGHT_MOTOR_MAX_VOLTAGE = 12;
    public static final double DEBOUNCER_SECONDS = 0.05;
    
    /* Sim: */
    public static class GripperSim {
        public static final double MOTORS_GEAR_RATIO = 0.1;
        public static final double MOTORS_MOMENT_OF_INERTIA = 0.1;
    }
}

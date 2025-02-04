package frc.robot.subsystems.gripper;

public class GripperConstants {
    public static final int LEFT_MOTOR_CURRENT_LIMIT = 30;
    public static final int RIGHT_MOTOR_CURRENT_LIMIT = 20;

    /* Gripper: */
    public static final double LEFT_MOTOR_MAX_VOLTAGE = 12;
    public static final double RIGHT_MOTOR_MAX_VOLTAGE = 12;
    public static final double DEBOUNCER_SECONDS = 0.05;

    /* Gripper Commands: */

    // Loading:
    public static final double LODING_MOTORS_VOLTAGE = 10;
    // Score L1:
    public static final double L1_RIGHT_MOTOR_VOLTAGE = 6;
    public static final double L1_LEFT_MOTOR_VOLTAGE = 6;

    // Score L3:
    public static final double L3_MOTORS_VOLTAGE = 0;
    
    /* Sim: */
    public static class GripperSim {
        public static final double MOTORS_GEAR_RATIO = 0;
        public static final double MOTORS_MOMENT_OF_INERTIA = 0;
    }
}

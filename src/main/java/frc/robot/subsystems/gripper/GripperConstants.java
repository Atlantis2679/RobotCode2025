package frc.robot.subsystems.gripper;

public class GripperConstants {
    /* Gripper: */
    public static final double OUTTAKE_MOTORS_MAX_VOLTAGE = 0;
    public static final double BACK_MOTOR_MAX_VOLTAGE = 0;
    public static final double DEBOUNCER_SECONDS = 0;

    /* Gripper Commands: */

    // Loading:
    public static final double LODING_OUTTAKE_MOTORS_VOLTAGE = 0;
    public static final double LOADING_BACK_MOTOR_VOLTAGE = 0;

    // Score L1:
    public static final double L1_RIGHT_OUTTAKE_MOTOR_VOLTAGE = 0;
    public static final double L1_LEFT_OUTTAKE_MOTOR_VOLTAGE = 0;
    public static final double L1_BACK_MOTOR_VOLTAGE = 0;

    // Score L3:
    public static final double L3_OUTTAKE_MOTORS_VOLTAGE = 0;
    public static final double L3_BACK_MOTOR_VOLTAGE = 0;
    
    /* Sim: */
    public static class GripperSim {
        public static final double OUTTAKE_MOTORS_GEAR_RATIO = 0;
        public static final double OUTTAKE_MOTORS_MOMENT_OF_INERTIA = 0;

        public static final double BACK_MOTOR_GEAR_RATIO = 0;
        public static final double BACK_MOTOR_MOMENT_OF_INERTIA = 0;
    }
}

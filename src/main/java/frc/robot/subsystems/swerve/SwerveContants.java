package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveContants {
    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_TURN = 12.8;
    public final static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    public final static double MAX_VOLTAGE = 11;
    public final static double MAX_MODULE_VELOCITY_MPS = 3.84; // for the max voltage

    public final static double TRACK_WIDTH_METERS = 0.518;
    public final static double TRACK_LENGTH_METERS = 0.518;
    public final static double TRACK_RADIUS_METERS = 0.366;

    public final static double MODULE_FL_ABSOLUTE_ANGLE_OFFSET_DEGREES = 149.94140625;
    public final static double MODULE_FR_ABSOLUTE_ANGLE_OFFSET_DEGREES = 123.662109375;
    public final static double MODULE_BL_ABSOLUTE_ANGLE_OFFSET_DEGREES = -53.96484375;
    public final static double MODULE_BR_ABSOLUTE_ANGLE_OFFSET_DEGREES = -10.8984375 + 180;

    public final static double MODULE_TURN_KP = 1.8 * 12;
    public final static double MODULE_TURN_KI = 0;
    public final static double MODULE_TURN_KD = 0;

    public static class DriverController {
        public final static double DRIVER_ACCELERATION_LIMIT_MPS = 8;
        public final static double DRIVER_ANGULAR_ACCELERATION_LIMIT_RPS = Math.toRadians(720);

        public final static double DRIVER_MAX_ANGULAR_VELOCITY_RPS = 4.5;

        public final static double SENSETIVE_TRANSLATION_MULTIPLIER = 0.3;
        public final static double SENSETIVE_ROTATION_MULTIPLIER = 0.3;
    }

    public class PathPlanner {
        public final static double FRICTION_WITH_CARPET = 1;//couldn't find it
        public final static double ROBOT_MASS_KG = 47;
        public final static double MOMENT_OF_INERTIA =  (1/12)*ROBOT_MASS_KG*(TRACK_LENGTH_METERS*TRACK_LENGTH_METERS+TRACK_WIDTH_METERS*TRACK_WIDTH_METERS);

        public final static double TRANSLATION_KP = 0;
        public final static double TRANSLATION_KI = 0;
        public final static double TRANSLATION_KD = 0;

        public final static double ROTATION_KP = 0;
        public final static double ROTATION_KI = 0;
        public final static double ROTATION_KD = 0;
    }

    public class RotateToAngle {
        public final static double KP = 0;
        public final static double KI = 0;
        public final static double KD = 0;
    }

    public class DriveToPose {
        public final static double MAX_VELOCITY_MPS = 1;
        public final static double MAX_ACCELERATION_MPS = 1;
        public final static double MAX_ANGULAR_VELOCITY_RPS = Math.toRadians(540);
        public final static double MAX_ANGULAR_ACCELERATION_RPS = Math.toRadians(620);
        public final static double GOAL_VELOCITY = 0;
        public final static boolean ROTATE_FAST = true;
    }
}

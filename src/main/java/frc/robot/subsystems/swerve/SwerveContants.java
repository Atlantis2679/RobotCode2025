package frc.robot.subsystems.swerve;

import edu.wpi.first.math.util.Units;

public class SwerveContants {
    public final static double GEAR_RATIO_DRIVE = 6.75;
    public final static double GEAR_RATIO_TURN = 12.8;
    public final static double WHEEL_RADIUS_METERS = Units.inchesToMeters(2);

    public final static double MAX_VOLTAGE = 11;
    public final static double MAX_MODULE_VELOCITY_MPS = 5; // for the max voltage
    public final static double MODULE_TEMPERATORE_WARNING_THRESHOLD = 0;

    public final static double TRACK_WIDTH_METERS = 0.76 + 8.5;
    public final static double TRACK_LENGTH_METERS = 0.76 + 8.5;
    public final static double TRACK_RADIUS_METERS = 0.366;


    public final static double MODULE_FL_ABSOLUTE_ANGLE_OFFSET_DEGREES = 155.390625;
    public final static double MODULE_FR_ABSOLUTE_ANGLE_OFFSET_DEGREES = -56.6015625+180;
    public final static double MODULE_BL_ABSOLUTE_ANGLE_OFFSET_DEGREES = -100.546875;
    public final static double MODULE_BR_ABSOLUTE_ANGLE_OFFSET_DEGREES = 37.96875+180;


    public final static double MODULE_TURN_KP = 1.8 * 12;
    public final static double MODULE_TURN_KI = 0;
    public final static double MODULE_TURN_KD = 0;

    static final double
        TRANSLATION_TOLERANCE_METERS = 0.02,
        ROTATION_TOLERANCE_DEGREES = 1,
        TRANSLATION_VELOCITY_TOLERANCE = 0.03,
        ROTATION_VELOCITY_TOLERANCE = 0.2;

    public static class DriverController {
        public final static double DRIVER_ACCELERATION_LIMIT_MPS = 8;
        public final static double DRIVER_ANGULAR_ACCELERATION_LIMIT_RPS = Math.toRadians(720);

        public final static double DRIVER_MAX_ANGULAR_VELOCITY_RPS = 4.5;

        public final static double SENSETIVE_TRANSLATION_MULTIPLIER = 0.3;
        public final static double SENSETIVE_ROTATION_MULTIPLIER = 0.3;
    }

    public class PathPlanner {
        public final static double FRICTION_WITH_CARPET = 1;//couldn't find it
        //
        public final static double ROBOT_MASS_KG = 47;
        public final static double MOMENT_OF_INERTIA =  (1/12)*ROBOT_MASS_KG*(TRACK_LENGTH_METERS*TRACK_LENGTH_METERS+TRACK_WIDTH_METERS*TRACK_WIDTH_METERS);

        //
        public final static double TRANSLATION_KP = 0.5;
        public final static double TRANSLATION_KI = 0;
        public final static double TRANSLATION_KD = 0;

        //
        public final static double ROTATION_KP = 1;
        public final static double ROTATION_KI = 0;
        public final static double ROTATION_KD = 0;
    }

    public class RotateToAngle {
        public final static double KP = 0;
        public final static double KI = 0;
        public final static double KD = 0;
    }

    public class DriveToPose {
        public final static double Y_KP = 2.2;//1.7
        public final static double Y_KI = 0;
        public final static double Y_KD = 0.75;

        public final static double X_KP = 2.2;//2.1
        public final static double X_KI = 0;
        public final static double X_KD = 0.75;

        public final static double ANGLE_KP = 1.5;//1.1
        public final static double ANGLE_KI = 0;
        public final static double ANGLE_KD = 0.1;

        public final static double MAX_VELOCITY_MPS = 0.5;
        public final static double MAX_ACCELERATION_MPS = 0.5;
        public final static double MAX_ANGULAR_VELOCITY_RPS = Math.toRadians(270);
        public final static double MAX_ANGULAR_ACCELERATION_RPS = Math.toRadians(620);
        public final static double GOAL_VELOCITY = 0;

        public final static boolean ROTATE_FAST = true;
    }

    public class AlignToReef {
        public static final double MIN_DISTANCE_TO_AMPALIGN = 1.5;        
    }
}

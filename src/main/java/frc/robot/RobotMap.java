package frc.robot;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.studica.frc.AHRS.NavXComType;

public class RobotMap {
    public static final ModuleType POWER_DISTRIBUATION_TYPE = ModuleType.kRev;

    public static final NavXComType NAVX_PORT = NavXComType.kMXP_SPI;
    public static final String FRONT_PHOTON_CAMERA_NAME = "FrontCam";
    public static final String BACK_LIMELIGHT_CAMERA_NAME = "limelight-back";
    
    public static final int GRIPPER_BEAM_BRAKE_ID = 0;

    public static final int FUNNEL_BEAM_BRAKE_ID = 9;

    public static final int PIVOT_ENCODER_ID = 1;

    public final static int LEDS_ID = 9;

    public static class Controllers {
        public static final int DRIVER_PORT = 0;
        public static final int OPERATOR_PORT = 1;
    }
       
    public static class CANBUS {
        public static final int FUNNEL_MOTOR_ID = 10;

        public static final int GRIPPER_LEFT_OUTTAKE_MOTOR_ID = 13;
        public static final int GRIPPER_RIGHT_OUTTAKE_MOTOR_ID = 14;
        public static final int GRIPPER_BACK_MOTOR_ID = 12;

        public static final int PIVOT_MOTOR_ID = 11;
    
        public class ModuleFL {
            public final static int DRIVE_MOTOR_ID = 20;
            public final static int TURN_MOTOR_ID = 21;
            public final static int ENCODER_ID = 50;
        }
    
        public class ModuleFR {
            public final static int DRIVE_MOTOR_ID = 22;
            public final static int TURN_MOTOR_ID = 23;
            public final static int ENCODER_ID = 51;
        }
    
        public class ModuleBL {
            public final static int DRIVE_MOTOR_ID = 26;
            public final static int TURN_MOTOR_ID = 27;
            public final static int ENCODER_ID = 52;
        }
    
        public class ModuleBR {
            public final static int DRIVE_MOTOR_ID = 24;
            public final static int TURN_MOTOR_ID = 25;
            public final static int ENCODER_ID = 53;
        }
    }
}

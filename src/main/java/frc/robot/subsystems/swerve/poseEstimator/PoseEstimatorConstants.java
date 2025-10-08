package frc.robot.subsystems.swerve.poseEstimator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseEstimatorConstants {
        public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT_RIGHT = new Transform3d(
                        new Translation3d(0.285, -0.19, 0.355),
                        new Rotation3d(Degrees.of(-0.76), Degrees.of(8.5), Degrees.of(-4)));

        public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT_LEFT = new Transform3d(
                        new Translation3d(0.31, 0.19, 0.22),
                        new Rotation3d(Degrees.of(-1), Degrees.of(-13), Degrees.of(0)));

        public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_BACK = new Transform3d(
                        new Translation3d(-0.27, 0, 0.54),
                        new Rotation3d(Degrees.of(0), Degrees.of(-23.5), Degrees.of(154)));

        public final static double VISION_TAG_ANBIGUITY_THRESHOLD = 0.3; // Need to calibrate
        public final static double VISION_MIN_TAG_DISTANCE_TO_POSE_METERS = 0.05; // Need to calibrate

        public final static double VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER = 3.5; // Need to calibrate
        public final static double VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER_WITHOUT_GYRO = 0.5; // Need to calibrate
        public final static double VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER = 2; // Need to calibrate

        public final static double STATE_TRUST_LEVEL_X = 2.5; // Need to calibrate
        public final static double STATE_TRUST_LEVEL_Y = 2.5; // Need to calibrate
        public final static double STATE_TRUST_LEVEL_ROTAION = 0.3; // Need to calibrate

        public final static double VISION_TRUST_LEVEL_THRESHOLD = 1; // Decrease in order to fillter more poses

        public final static double MAX_VISION_Z_OFF = 0.3;
        public final static double MAX_ROTATION_OFF = Math.toDegrees(20);
}

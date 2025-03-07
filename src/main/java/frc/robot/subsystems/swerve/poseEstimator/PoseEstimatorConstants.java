package frc.robot.subsystems.swerve.poseEstimator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseEstimatorConstants {
    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT = new Transform3d(
            new Translation3d(0.285, -0.195, 0.36),
            new Rotation3d(Degrees.of(2), Degrees.of(11), Degrees.of(0)));

    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_BACK = new Transform3d(
            new Translation3d(-0.27, 0, 0.54),
            new Rotation3d(Degrees.of(0), Degrees.of(-23.5), Degrees.of(154)));

    public final static double VISION_TAG_ANBIGUITY_THRESHOLD = 0.3; // Need to calibrate
    public final static double VISION_MIN_TAG_DISTANCE_TO_POSE_METERS = 0.05; // Need to calibrate

    public final static double VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER = 3.5; // Need to calibrate
    public final static double VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER = 2; // Need to calibrate

    public final static double STATE_TRUST_LEVEL_X = 2.5; // Need to calibrate
    public final static double STATE_TRUST_LEVEL_Y = 2.5; // Need to calibrate
    public final static double STATE_TRUST_LEVEL_ROTAION = 0.3; // Need to calibrate

    public final static double MAX_VISION_Z_OFF = 0.3;
    public final static double MAX_ROTATION_OFF = Math.toDegrees(20);
}

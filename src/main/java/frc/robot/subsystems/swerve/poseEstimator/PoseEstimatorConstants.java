// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseEstimatorConstants {
    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT = new Transform3d(
            new Translation3d(0.245, 0.13, 0.51),
            new Rotation3d(Degrees.of(0), Degrees.of(-28.6), Degrees.of(180)));

    public final static double VISION_THRESHOLD_DISTANCE_M = 1.5;

    public final static double STATE_TRUST_LEVEL_X = 0.9;
    public final static double STATE_TRUST_LEVEL_Y = 0.9;
    public final static double STATE_TRUST_LEVEL_Z = 0.9;

    public final static double VISION_TAG_AMBIGUITY_MIN_VALUE = 0.00005;
    public final static double VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER = 1;
    public final static double VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER = 1;
}

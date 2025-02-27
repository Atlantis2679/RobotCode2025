// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve.poseEstimator;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PoseEstimatorConstants {
    // public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT = new Transform3d(
    //         new Translation3d(0.245, 0.13, 0.51),
    //         new Rotation3d(Degrees.of(0), Degrees.of(-12), Degrees.of(0)));

    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT = new Transform3d(
            new Translation3d(0.20, 0.31, 0.385),
            new Rotation3d(Degrees.of(0), Degrees.of(12), Degrees.of(0)));

    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_BACK = new Transform3d(
            new Translation3d(0, -40, 0.59),
            new Rotation3d(Degrees.of(0), Degrees.of(-48), Degrees.of(180)));

    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_RIGHT = new Transform3d(
            new Translation3d(0.34, 0, 0.45),
            new Rotation3d(Degrees.of(24.95), Degrees.of(-38), Degrees.of(-118)));

    public final static Transform3d ROBOT_TO_CAMERA_TRANSFORM_PHOTON_LEFT = new Transform3d(
            new Translation3d(-0.31, 0, 0.465),
            new Rotation3d(Degrees.of(24.95), Degrees.of(-38), Degrees.of(118)));

    public final static double VISION_MAX_TAG_ANBIGUITY_THRESHOLD = 0.7; // Need to check
    public final static double VISION_MIN_TAG_DISTANCE_TO_POSE_METERS = 0.05; // Need to check

    public final static double VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER = 1; // Need to check
    public final static double VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER = 1; // Need to check

    public final static double STATE_TRUST_LEVEL_X = 0.9; // Need to check
    public final static double STATE_TRUST_LEVEL_Y = 0.9; // Need to check
    public final static double STATE_TRUST_LEVEL_Z = 0.9; // Need to check
}

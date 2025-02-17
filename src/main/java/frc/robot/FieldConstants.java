package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final Pose2d[] REEF_PLACE_POSES = new Pose2d[]{
    new Pose2d(
    new Translation2d(3.13, 3.79),
    new Rotation2d(Math.toRadians(0))
   ),
    new Pose2d(
    new Translation2d(3.13, 4.79),
    new Rotation2d(Math.toRadians(0))
   ),
    new Pose2d(
    new Translation2d(3.670, 5.107),
    new Rotation2d(Math.toRadians(-60))//works and is 3 oclock
    ),
     new Pose2d(
    new Translation2d(3.939, 5.286),
    new Rotation2d(Math.toRadians(-60))
    ),
    new Pose2d(
    new Translation2d(5.095, 5.336),
    new Rotation2d(Math.toRadians(-120))
    ),
    new Pose2d(
    new Translation2d(5.355, 5.167),
    new Rotation2d(Math.toRadians(-120))
    ),
    new Pose2d(
    new Translation2d(5.913, 4.180),
    new Rotation2d(Math.toRadians(-180))
    ),
    new Pose2d(
    new Translation2d(5.913, 3.870),
    new Rotation2d(Math.toRadians(-180))
    ),
    new Pose2d(
    new Translation2d(5.345, 2.903),
    new Rotation2d(Math.toRadians(120))
    ),
    new Pose2d(
    new Translation2d(5.066, 2.744),
    new Rotation2d(Math.toRadians(120))
    ),
    new Pose2d(
    new Translation2d(3.949, 2.734),
    new Rotation2d(Math.toRadians(60))
    ),
    new Pose2d(
    new Translation2d(3.640, 2.873),
    new Rotation2d(Math.toRadians(60))
    )
    };
}

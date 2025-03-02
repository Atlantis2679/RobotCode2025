package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    // public static final Pose2d[] REEF_FACE_CENTER = new Pose2d[] {
    //     new Pose2d(
    //         new Translation2d(3.159, 4.019),
    //         new Rotation2d(Math.toRadians(0))
    //     ),
    //     new Pose2d(
    //         new Translation2d(3.848, 5.146),
    //         new Rotation2d(Math.toRadians(-60))//works and is 3 oclock
    //     ),
    //     new Pose2d(
    //         new Translation2d(5.131, 5.122),
    //         new Rotation2d(Math.toRadians(-120))
    //     ),
    //     new Pose2d(
    //         new Translation2d(5.772, 4.019),
    //         new Rotation2d(Math.toRadians(-180))
    //     ),
    //     new Pose2d(
    //         new Translation2d(5.131, 2.928),
    //         new Rotation2d(Math.toRadians(120))
    //     ),
    //     new Pose2d(
    //         new Translation2d(3.824, 2.928),
    //         new Rotation2d(Math.toRadians(60))
    //     ) 
    // };

    public static final Pose2d[] REEF_RIGHT_BRANCHES_POSES = new Pose2d[] {
        new Pose2d(
            new Translation2d(3.15, 3.85),
            new Rotation2d(Math.toRadians(0))
        ),
        new Pose2d(
            new Translation2d(3.668, 5.086),
            new Rotation2d(Math.toRadians(-60))//works and is 3 oclock
        ),
        new Pose2d(
            new Translation2d(5.023, 5.254),
            new Rotation2d(Math.toRadians(-120))
        ),
        new Pose2d(
            new Translation2d(5.814, 4.175),
            new Rotation2d(Math.toRadians(-180))
        ),
        new Pose2d(
            new Translation2d(5.307, 2.928),
            new Rotation2d(Math.toRadians(120))
        ),
        new Pose2d(
            new Translation2d(3.956, 2.796),
            new Rotation2d(Math.toRadians(60))
        )
    };

    public static final Pose2d[] REEF_LEFT_BRANCHES_POSES = new Pose2d[] {
        new Pose2d(
            new Translation2d(3.15, 4.20),
            new Rotation2d(Math.toRadians(0))
        ),
        new Pose2d(
            new Translation2d(3.943, 5.280),
            new Rotation2d(Math.toRadians(-60))
        ),
        new Pose2d(
            new Translation2d(5.316, 5.122),
            new Rotation2d(Math.toRadians(-120))
        ),
        new Pose2d(
            new Translation2d(5.850, 3.887),
            new Rotation2d(Math.toRadians(-180))
        ),
        new Pose2d(
            new Translation2d(5.040, 2.760),
            new Rotation2d(Math.toRadians(120))
        ),
        new Pose2d(
            new Translation2d(3.666, 2.918),
            new Rotation2d(Math.toRadians(60))
        )
    };
}

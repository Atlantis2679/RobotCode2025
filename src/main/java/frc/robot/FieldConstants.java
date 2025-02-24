package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FieldConstants {
    public static final Pose2d[] REEF_FACE_CENTER = new Pose2d[] {
        new Pose2d(
            new Translation2d(3.159, 4.019),
            new Rotation2d(Math.toRadians(0))
        ),
        new Pose2d(
            new Translation2d(3.848, 5.146),
            new Rotation2d(Math.toRadians(-60))//works and is 3 oclock
        ),
        new Pose2d(
            new Translation2d(5.131, 5.122),
            new Rotation2d(Math.toRadians(-120))
        ),
        new Pose2d(
            new Translation2d(5.772, 4.019),
            new Rotation2d(Math.toRadians(-180))
        ),
        new Pose2d(
            new Translation2d(5.131, 2.928),
            new Rotation2d(Math.toRadians(120))
        ),
        new Pose2d(
            new Translation2d(3.824, 2.928),
            new Rotation2d(Math.toRadians(60))
        ) 
    };

    public static final Pose2d[] REEF_RIGHT_BRANCHES_POSES = new Pose2d[] {
        new Pose2d(
            new Translation2d(3.159, 3.869),
            new Rotation2d(Math.toRadians(0))
        ),
        new Pose2d(
            new Translation2d(3.715, 5.049),
            new Rotation2d(Math.toRadians(-60))//works and is 3 oclock
        ),
        new Pose2d(
            new Translation2d(4.992, 5.224),
            new Rotation2d(Math.toRadians(-120))
        ),
        new Pose2d(
            new Translation2d(5.772, 4.191),
            new Rotation2d(Math.toRadians(-180))
        ),
        new Pose2d(
            new Translation2d(5.245, 2.982),
            new Rotation2d(Math.toRadians(120))
        ),
        new Pose2d(
            new Translation2d(3.978, 2.826),
            new Rotation2d(Math.toRadians(60))
        )
    };

    public static final Pose2d[] REEF_LEFT_BRANCHES_POSES = new Pose2d[] {
        new Pose2d(
            new Translation2d(3.159, 4.161),
            new Rotation2d(Math.toRadians(0))
        ),
        new Pose2d(
            new Translation2d(3.988, 5.215),
            new Rotation2d(Math.toRadians(-60))
        ),
        new Pose2d(
            new Translation2d(5.265, 5.068),
            new Rotation2d(Math.toRadians(-120))
        ),
        new Pose2d(
            new Translation2d(5.772, 3.898),
            new Rotation2d(Math.toRadians(-180))
        ),
        new Pose2d(
            new Translation2d(4.973, 2.865),
            new Rotation2d(Math.toRadians(120))
        ),
        new Pose2d(
            new Translation2d(3.695, 3.001),
            new Rotation2d(Math.toRadians(60))
        )
    };
}

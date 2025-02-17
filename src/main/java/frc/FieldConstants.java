package frc;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double
    FIELD_WIDTH_METERS = FlippingUtil.fieldSizeY,
    FIELD_LENGTH_METERS = FlippingUtil.fieldSizeX;

    public static class Reef {
        public static final double faceLength = Units.inchesToMeters(36.792600);
        public static final Translation2d center =
            new Translation2d(Units.inchesToMeters(176.746), FIELD_WIDTH_METERS / 2.0);
        public static final double faceToZoneLine =
            Units.inchesToMeters(12);

        public static final Pose2d[] centerFaces =
            new Pose2d[6]; // Starting facing the driver station in clockwise order
        public static final List<Map<ReefLevel, Pose3d>> branchPositions =
            new ArrayList<>(); // Starting at the right branch facing the driver station in clockwise
        public static final List<Map<ReefLevel, Pose2d>> branchPositions2d = new ArrayList<>();

        static {
        // Initialize faces
        var aprilTagLayout = AprilTagLayoutType.OFFICIAL.getLayout();
        centerFaces[0] = aprilTagLayout.getTagPose(18).get().toPose2d();
        centerFaces[1] = aprilTagLayout.getTagPose(19).get().toPose2d();
        centerFaces[2] = aprilTagLayout.getTagPose(20).get().toPose2d();
        centerFaces[3] = aprilTagLayout.getTagPose(21).get().toPose2d();
        centerFaces[4] = aprilTagLayout.getTagPose(22).get().toPose2d();
        centerFaces[5] = aprilTagLayout.getTagPose(17).get().toPose2d();

        // Initialize branch positions
        for (int face = 0; face < 6; face++) {
            Map<ReefLevel, Pose3d> fillRight = new HashMap<>();
            Map<ReefLevel, Pose3d> fillLeft = new HashMap<>();
            Map<ReefLevel, Pose2d> fillRight2d = new HashMap<>();
            Map<ReefLevel, Pose2d> fillLeft2d = new HashMap<>();
            for (var level : ReefLevel.values()) {
            Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
            double adjustX = Units.inchesToMeters(30.738);
            double adjustY = Units.inchesToMeters(6.469);

            var rightBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));
            var leftBranchPose =
                new Pose3d(
                    new Translation3d(
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getX(),
                        poseDirection
                            .transformBy(new Transform2d(adjustX, -adjustY, new Rotation2d()))
                            .getY(),
                        level.height),
                    new Rotation3d(
                        0,
                        Units.degreesToRadians(level.pitch),
                        poseDirection.getRotation().getRadians()));

            fillRight.put(level, rightBranchPose);
            fillLeft.put(level, leftBranchPose);
            fillRight2d.put(level, rightBranchPose.toPose2d());
            fillLeft2d.put(level, leftBranchPose.toPose2d());
            }
            branchPositions.add(fillRight);
            branchPositions.add(fillLeft);
            branchPositions2d.add(fillRight2d);
            branchPositions2d.add(fillLeft2d);
        }
        }
  }
        
  public enum ReefLevel {
    L1(Units.inchesToMeters(25.0), 0),
    L2(Units.inchesToMeters(31.875 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L3(Units.inchesToMeters(47.625 - Math.cos(Math.toRadians(35.0)) * 0.625), -35),
    L4(Units.inchesToMeters(72), -90);

    ReefLevel(double height, double pitch) {
      this.height = height;
      this.pitch = pitch; // Degrees
    }

    public static ReefLevel fromLevel(int level) {
      return Arrays.stream(values())
          .filter(height -> height.ordinal() == level)
          .findFirst()
          .orElse(L4);
    }

    public final double height;
    public final double pitch;
  }
    // public static final int REEF_CLOCK_POSITIONS = 6;
    // public static final Rotation2d REEF_CLOCK_POSITION_DIFFERENCE = Rotation2d.fromDegrees(Conversions.DEGREES_PER_ROTATIONS / REEF_CLOCK_POSITIONS);
    // public static final Rotation2d[] REEF_CLOCK_ANGLES = ReefClockPosition.getClockAngles();
    // public static final Translation2d BLUE_REEF_CENTER_TRANSLATION = new Translation2d(4.48945, FIELD_WIDTH_METERS / 2);


    // public enum ReefSide {
    //     LEFT(false),
    //     RIGHT(true);

    //     public final boolean doesFlipYTransformWhenFacingDriverStation;

    //     ReefSide(boolean doesFlipYTransformWhenFacingDriverStation) {
    //         this.doesFlipYTransformWhenFacingDriverStation = doesFlipYTransformWhenFacingDriverStation;
    //     }

    //     public boolean shouldFlipYTransform(ReefClockPosition reefClockPosition) {
    //         return doesFlipYTransformWhenFacingDriverStation ^ reefClockPosition.isFacingDriverStation; // In Java, ^ acts as an XOR (exclusive OR) operator, which fits in this case
    //     }
    // }
    // public enum ReefClockPosition {
    //     REEF_12_OCLOCK(true, 3),
    //     REEF_2_OCLOCK(true, 2),
    //     REEF_4_OCLOCK(true, 1),
    //     REEF_6_OCLOCK(true, 0),
    //     REEF_8_OCLOCK(true, 5),
    //     REEF_10_OCLOCK(true, 4);

    //     public final Rotation2d clockAngle;
    //     public final boolean isFacingDriverStation;
    //     public final int qDashboardOrder;

    //     ReefClockPosition(boolean isFacingDriverStation, int qDashboardOrder) {
    //         this.clockAngle = calculateClockAngle();
    //         this.isFacingDriverStation = isFacingDriverStation;
    //         this.qDashboardOrder = qDashboardOrder;
    //     }
    //     public static Rotation2d[] getClockAngles() {
    //         final Rotation2d[] clockAngles = new Rotation2d[ReefClockPosition.values().length];
    //         for (int i = 0; i < clockAngles.length; i++)
    //             clockAngles[i] = ReefClockPosition.values()[i].clockAngle;

    //         return clockAngles;
    //     }

    //     private Rotation2d calculateClockAngle() {
    //         return REEF_CLOCK_POSITION_DIFFERENCE.times(-ordinal());
    //     }
    // }
}

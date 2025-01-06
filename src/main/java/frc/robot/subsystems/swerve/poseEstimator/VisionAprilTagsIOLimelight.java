package frc.robot.subsystems.swerve.poseEstimator;

import frc.lib.logfields.LogFieldsTable;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightResults;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.utils.LimelightHelpers.PoseEstimate;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionAprilTagsIOLimelight extends VisionAprilTagsIO {

    private LimelightResults limelightResults;
    // private Pose3d limelightPose3d;
    private final String limelightName;

    public VisionAprilTagsIOLimelight(LogFieldsTable fieldsTable, String limelightName) {
        super(fieldsTable);
        this.limelightName = limelightName;
        LimelightHelpers.setCameraPose_RobotSpace(limelightName,
                PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getTranslation().getX(),
                PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getTranslation().getY(),
                PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getTranslation().getZ(),
                Math.toDegrees(PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getRotation().getX()),
                Math.toDegrees(PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getRotation().getY()),
                Math.toDegrees(PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_LIMELIGHT_BACK.getRotation().getZ()));
    }

    @Override
    public void periodicBeforeFields() {
        limelightResults = LimelightHelpers.getLatestResults(limelightName);
    }

    @Override
    protected double getCameraTimestampSeconds() {
        PoseEstimate data = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (data != null) {
            return limelightResults != null
                    ? LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName).timestampSeconds
                    : 0;
        } else {
            return 0;
        }
    }

    @Override
    protected Pose3d getRobotPose() {
        // Pose3d estimate = new Pose3d(limelightResults.pose);
        // return estimate != null ? estimate : new Pose3d();
        return limelightResults.getBotPose3d() != null ? limelightResults.getBotPose3d_wpiBlue() : new Pose3d();
    }

    @Override
    protected boolean getHasNewRobotPose() {
        return limelightResults != null && limelightResults.botpose_tagcount > 0;
    }

    @Override
    protected int getVisibleTargetCount() {
        return limelightResults != null ? (int) limelightResults.botpose_tagcount : 0;
    }

    @Override
    protected Transform3d[] getTargetsPosesInRobotSpace() {
        if (limelightResults == null) {
            return new Transform3d[] {};
        }
        LimelightTarget_Fiducial[] fiducials = limelightResults.targets_Fiducials;
        Transform3d[] targetsArr = new Transform3d[(int) limelightResults.botpose_tagcount];
        for (int i = 0; i < (int) limelightResults.botpose_tagcount; i++) {
            targetsArr[i] = new Transform3d(new Pose3d(), fiducials[i].getTargetPose_RobotSpace());
        }

        return targetsArr;
    }
}

package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.utils.NetworkAlertsManager;
import frc.robot.subsystems.swerve.poseEstimator.io.VisionAprilTagsIO;
import frc.robot.subsystems.swerve.poseEstimator.io.VisionAprilTagsIOPhoton;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonUtils;

import static frc.robot.RobotMap.*;
import static frc.robot.subsystems.swerve.poseEstimator.PoseEstimatorConstants.*;

public class PoseEstimatorWithVision {
    private final Map<String, VisionAprilTagsIO> visionCameras = new HashMap<>();
    private final SwerveDrivePoseEstimator poseEstimator;
    private final LogFieldsTable fieldsTable;

    public PoseEstimatorWithVision(LogFieldsTable fieldsTable, Rotation2d currentAngle,
            SwerveModulePosition[] positions, SwerveDriveKinematics swerveKinematics) {

        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeWelded.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException();
        }

        if (Robot.isReal()) {
            visionCameras.put(FRONT_PHOTON_CAMERA_NAME,
                    new VisionAprilTagsIOPhoton(fieldsTable, FRONT_PHOTON_CAMERA_NAME, fieldLayout,
                            PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT));

        // visionCameras.put(BACK_PHOTON_CAMERA_NAME,
        //         new VisionAprilTagsIOPhoton(fieldsTable, BACK_PHOTON_CAMERA_NAME, fieldLayout,
        //                 PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_BACK));

        // visionCameras.put(LEFT_PHOTON_CAMERA_NAME,
        //         new VisionAprilTagsIOPhoton(fieldsTable, LEFT_PHOTON_CAMERA_NAME, fieldLayout,
        //                 PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_LEFT));

        // visionCameras.put(RIGHT_PHOTON_CAMERA_NAME,
        //         new VisionAprilTagsIOPhoton(fieldsTable, RIGHT_PHOTON_CAMERA_NAME, fieldLayout,
        //                 PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_RIGHT));

        this.fieldsTable = fieldsTable;

        visionCameras.forEach((name, visionIO) -> {
            NetworkAlertsManager.addErrorAlert(name + " Camera Is Discconected!",
                    () -> !visionIO.isCameraConnected.getAsBoolean());
        });

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                currentAngle,
                positions,
                new Pose2d(),
                VecBuilder.fill(STATE_TRUST_LEVEL_X, STATE_TRUST_LEVEL_Y, STATE_TRUST_LEVEL_ROTAION),
                VecBuilder.fill(1, 1, 1));
    }

    public void update(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions) {
        poseEstimator.update(gyroMeasurmentCCW, modulesPositions);

        visionCameras.forEach((cameraName, visionIO) -> {
            LogFieldsTable cameraFieldsTable = fieldsTable.getSubTable(cameraName);
            Pose3d[] poses = visionIO.posesEstimates.get();
            for (int i = 0; i < poses.length; i++) {
                Pose3d poseEstimate = poses[i].transformBy(visionIO.getRobotToCameraTransform().inverse());
                Pose3d[] tagsPoses = visionIO.tagsPoses.get()[i];
                double[] tagsAmbiguities = visionIO.tagsAmbiguities.get()[i];
                double cameraTimestampSeconds = visionIO.cameraTimestampsSeconds.get()[i];

                cameraFieldsTable.recordOutput("Pose3d unfiltered", poseEstimate);

                double trustLevel = caculatePoseTrustLevel(
                        tagsPoses,
                        tagsAmbiguities,
                        poseEstimate);

                cameraFieldsTable.recordOutput("trust level unfiltered", Math.random());
                
                if (trustLevel == -1)
                    continue;

                trustLevel = trustLevel * trustLevel + trustLevel;

                if (!isOnField(poseEstimate))
                    continue;

                cameraFieldsTable.recordOutput("Pose3d", poseEstimate);
                cameraFieldsTable.recordOutput("Pose2d", poseEstimate.toPose2d());
                cameraFieldsTable.recordOutput("tagsPoses", tagsPoses);
                cameraFieldsTable.recordOutput("tagsAmbiguities", tagsAmbiguities);
                cameraFieldsTable.recordOutput("TrustLevel", trustLevel);

                double visionRotationTrustLevel = trustLevel * VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER;
                double visionTranslationTrustLevel = trustLevel * VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER;

                poseEstimator.addVisionMeasurement(
                        poseEstimate.toPose2d(), cameraTimestampSeconds, VecBuilder.fill(
                                visionTranslationTrustLevel,
                                visionTranslationTrustLevel,
                                visionRotationTrustLevel));
            }
        });
    }

    public void resetPosition(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroMeasurmentCCW, modulesPositions, newPose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private static double caculatePoseTrustLevel(Pose3d[] tagsPoses, double[] tagsAmbiguitys,
            Pose3d estimatedRobotPose) {
        double trustLevel = 0;
        for (int i = 0; i < tagsPoses.length; i++) {
            if (tagsAmbiguitys[i] <= VISION_TAG_ANBIGUITY_THRESHOLD) {
                double tagEstimatedDistanceToPose = Math.max(
                        PhotonUtils.getDistanceToPose(estimatedRobotPose.toPose2d(), tagsPoses[i].toPose2d()),
                        VISION_MIN_TAG_DISTANCE_TO_POSE_METERS);
                trustLevel += (1 / tagEstimatedDistanceToPose);
            }
        }
        return trustLevel != 0 ? 1 / trustLevel : -1;
    }

    private static boolean isOnField(Pose3d pose) {
        return (pose.getX() < FieldConstants.FIELD_LENGTH || pose.getX() > 0)
                && (pose.getY() < FieldConstants.FIELD_WIDTH || pose.getY() > 0)
                && Math.abs(pose.getZ()) < MAX_VISION_Z_OFF
                && Math.abs(pose.getRotation().getX()) < MAX_ROTATION_OFF
                && Math.abs(pose.getRotation().getY()) < MAX_ROTATION_OFF;
    }
}

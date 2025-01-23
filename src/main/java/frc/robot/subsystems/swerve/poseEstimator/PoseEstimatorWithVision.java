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
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025Reefscape.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
            throw new RuntimeException();
        }

        visionCameras.put(FRONT_PHOTON_CAMERA_NAME,
                new VisionAprilTagsIOPhoton(fieldsTable, FRONT_PHOTON_CAMERA_NAME, fieldLayout, PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT));

        this.fieldsTable = fieldsTable;

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                currentAngle,
                positions,
                new Pose2d(),
                VecBuilder.fill(STATE_TRUST_LEVEL_X, STATE_TRUST_LEVEL_Y, STATE_TRUST_LEVEL_Z),
                VecBuilder.fill(1, 1, 1));
    }

    public void update(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions) {
        poseEstimator.update(gyroMeasurmentCCW, modulesPositions);

        visionCameras.forEach((cameraName, visionIO) -> {
            Pose3d[] poses = visionIO.posesEstimates.get();
            for (int i = 0; i < poses.length; i++) {
                LogFieldsTable cameraFieldsTable = fieldsTable.getSubTable(cameraName);
                Pose3d poseEstimate = poses[i];
                double trustLevel = caculatePoseTrustLevel(
                    visionIO.tagsPoses.get()[i],
                    visionIO.tagsAmbiguitys.get()[i],
                    poseEstimate
                );
                if(trustLevel == -1) continue;
                cameraFieldsTable.recordOutput("Pose3d", poseEstimate);
                cameraFieldsTable.recordOutput("Pose2d", poseEstimate.toPose2d());
                cameraFieldsTable.recordOutput("tagsPoses", visionIO.tagsPoses.get()[i]);
                cameraFieldsTable.recordOutput("tagsAmbiguitys", visionIO.tagsAmbiguitys.get()[i]);
                cameraFieldsTable.recordOutput("trustLevel", trustLevel);
                    
                double visionRotationTrustLevel = trustLevel * VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER;
                double visionTranslationTrustLevel = trustLevel * VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER;
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTranslationTrustLevel, visionTranslationTrustLevel, visionRotationTrustLevel));
        
                poseEstimator.addVisionMeasurement(
                        poseEstimate.toPose2d(),
                        visionIO.cameraTimestampsSeconds.get()[i]);
            }
        });
    }

    public void resetPosition(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroMeasurmentCCW, modulesPositions, newPose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private static double caculatePoseTrustLevel(Pose3d[] tagsPoses, double[] tagsAmbiguitys, Pose3d estimatedRobotPose) {
        double trustLevel = 0;
        for (int i = 0; i < tagsPoses.length; i++) {
            if(tagsAmbiguitys[i] <= VISION_MAX_TAG_ANBIGUITY_THRESHOLD && tagsAmbiguitys[i] >= 0) {
                double tagEstimatedDistanceToPose = PhotonUtils.getDistanceToPose(estimatedRobotPose.toPose2d(), tagsPoses[i].toPose2d());
                if(tagEstimatedDistanceToPose < VISION_MIN_TAG_DISTANCE_TO_POSE_METERS) {
                    tagEstimatedDistanceToPose = VISION_MIN_TAG_DISTANCE_TO_POSE_METERS;
                }
                trustLevel += (1 / tagEstimatedDistanceToPose);
            }
        }
        return trustLevel != 0 ? 1 / trustLevel : -1;
    }
}

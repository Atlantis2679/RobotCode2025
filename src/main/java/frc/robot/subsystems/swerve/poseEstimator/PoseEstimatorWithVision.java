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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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

    private boolean ignoreFarEstimates = false;

    public PoseEstimatorWithVision(LogFieldsTable fieldsTable, Rotation2d currentAngle,
            SwerveModulePosition[] positions, SwerveDriveKinematics swerveKinematics) {
        new Trigger(DriverStation::isDisabled)
                .onTrue(Commands.runOnce(() -> ignoreFarEstimates = false).ignoringDisable(true));
        new Trigger(DriverStation::isEnabled).onTrue(Commands.runOnce(() -> ignoreFarEstimates = true).ignoringDisable(true));

        AprilTagFieldLayout fieldLayout;
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
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
            new Pose2d();
            Pose3d[] poses = visionIO.posesEstimates.get();
            for (int i = 0; i < poses.length; i++) {
                LogFieldsTable cameraFieldsTable = fieldsTable.getSubTable(cameraName);
                Pose3d poseEstimate = poses[i];
                double poseAmbiguity = caculatePoseAmbiguitys(visionIO.tagsAmbiguitys.get()[i]);
                cameraFieldsTable.recordOutput("Pose3d", poseEstimate);
                cameraFieldsTable.recordOutput("Pose2d", poseEstimate.toPose2d());
                cameraFieldsTable.recordOutput("tagsPoses", visionIO.tagsPoses.get()[i]);
                cameraFieldsTable.recordOutput("tagsAmbiguitys", visionIO.tagsAmbiguitys.get()[i]);
                cameraFieldsTable.recordOutput("poseAmbiguity", poseAmbiguity);

                double visionToEstimateDifference = PhotonUtils.getDistanceToPose(
                        poseEstimate.toPose2d(),
                        poseEstimator.getEstimatedPosition());

                cameraFieldsTable.recordOutput("diff",
                        PoseEstimatorConstants.VISION_THRESHOLD_DISTANCE_M - visionToEstimateDifference);
                    
                double visionRotationTrustLevel = poseAmbiguity * VISION_ROTATION_TRUST_LEVEL_MULTIPLAYER;
                double visionTranslationTrustLevel = poseAmbiguity * VISION_TRANSLATION_TRUST_LEVEL_MULTIPLAYER;
                poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(visionTranslationTrustLevel, visionTranslationTrustLevel, visionRotationTrustLevel));
        
                if (!ignoreFarEstimates
                        || visionToEstimateDifference < PoseEstimatorConstants.VISION_THRESHOLD_DISTANCE_M) {
                    poseEstimator.addVisionMeasurement(
                            poseEstimate.toPose2d(),
                            visionIO.cameraTimestampsSeconds.get()[i]);
                }
            }
        });
    }

    public void resetPosition(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions, Pose2d newPose) {
        poseEstimator.resetPosition(gyroMeasurmentCCW, modulesPositions, newPose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    private static double caculatePoseAmbiguitys(double[] tagsAmbiguitys) {
        double multiplay = 1;
        for(double tagAmbiguity : tagsAmbiguitys) {
            multiplay *= tagAmbiguity;
        }
        return Math.sqrt(multiplay);
    }
}

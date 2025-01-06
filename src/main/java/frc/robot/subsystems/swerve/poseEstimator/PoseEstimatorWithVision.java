package frc.robot.subsystems.swerve.poseEstimator;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.logfields.LogFieldsTable;

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
        // new Trigger(DriverStation::isEnabled)
        // .whileTrue(Commands.runOnce(() -> ignoreFarEstimates =
        // true).ignoringDisable(true));

        try {
            AprilTagFieldLayout tagsLayout = AprilTagFieldLayout
                    .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);

            visionCameras.put("Front Photon",
                    new VisionAprilTagsIOPhoton(
                            fieldsTable.getSubTable("Front Photon"),
                            FRONT_PHOTON_CAMERA_NAME, tagsLayout));
            visionCameras.put("Back Limelight",
                    new VisionAprilTagsIOLimelight(
                            fieldsTable.getSubTable("Back Limelight"),
                            BACK_LIMELIGHT_CAMERA_NAME));
        } catch (IOException e) {
            DriverStation.reportError("AprilTagFieldLayout blew up", e.getStackTrace());
            throw new RuntimeException(e);
        }

        this.fieldsTable = fieldsTable;

        poseEstimator = new SwerveDrivePoseEstimator(
                swerveKinematics,
                currentAngle,
                positions,
                new Pose2d(),
                VecBuilder.fill(STATE_TRUST_LEVEL_X, STATE_TRUST_LEVEL_Y, STATE_TRUST_LEVEL_Z),
                VecBuilder.fill(VISION_TRUST_LEVEL_X, VISION_TRUST_LEVEL_Y, VISION_TRUST_LEVEL_Z));
    }

    public void update(Rotation2d gyroMeasurmentCCW, SwerveModulePosition[] modulesPositions) {
        poseEstimator.update(gyroMeasurmentCCW, modulesPositions);

        visionCameras.forEach((cameraName, visionIO) -> {
            if (visionIO.hasNewRobotPose.getAsBoolean()) {
                LogFieldsTable cameraFieldsTable = fieldsTable.getSubTable(cameraName);
                Pose3d poseEstimate = visionIO.poseEstimate.get();
                cameraFieldsTable.recordOutput("Pose3d", poseEstimate);
                cameraFieldsTable.recordOutput("Pose2d", poseEstimate.toPose2d());

                Transform3d[] targetsTransforms = visionIO.targetsPosesInRobotSpace.get();
                Pose3d[] targetPoses = new Pose3d[targetsTransforms.length];
                for (int i = 0; i < targetsTransforms.length; i++) {
                    targetPoses[i] = poseEstimate.plus(new Transform3d(targetsTransforms[i].getTranslation(),
                            targetsTransforms[i].getRotation().unaryMinus()));
                }
                cameraFieldsTable.recordOutput("targetPoses", targetPoses);

                double visionToEstimateDifference = PhotonUtils.getDistanceToPose(
                        visionIO.poseEstimate.get().toPose2d(),
                        poseEstimator.getEstimatedPosition());

                cameraFieldsTable.recordOutput("diff",
                        PoseEstimatorConstants.VISION_THRESHOLD_DISTANCE_M - visionToEstimateDifference);

                if (!ignoreFarEstimates
                        || visionToEstimateDifference < PoseEstimatorConstants.VISION_THRESHOLD_DISTANCE_M) {
                    poseEstimator.addVisionMeasurement(
                            visionIO.poseEstimate.get().toPose2d(),
                            visionIO.cameraTimestampSeconds.getAsDouble());
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
}

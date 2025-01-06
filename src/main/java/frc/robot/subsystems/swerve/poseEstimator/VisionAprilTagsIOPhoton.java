package frc.robot.subsystems.swerve.poseEstimator;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.logfields.LogFieldsTable;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera camera;
    private PhotonPipelineResult photonPipelineResult;
    Optional<EstimatedRobotPose> photonEstimatorResult;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, String cameraName, AprilTagFieldLayout tagLayout) {
        super(fieldsTable);

        camera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera,
                PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT);
    }

    @Override
    public void periodicBeforeFields() {
        photonPipelineResult = camera.getLatestResult();
        photonEstimatorResult = photonPoseEstimator.update(photonPipelineResult);
    }

    @Override
    protected double getCameraTimestampSeconds() {
        return photonPipelineResult != null ? photonPipelineResult.getTimestampSeconds() : 0;
    }

    @Override
    protected Pose3d getRobotPose() {
        EstimatedRobotPose estimate = photonEstimatorResult != null
                ? photonEstimatorResult.orElse(null)
                : null;
        return estimate != null ? estimate.estimatedPose : new Pose3d();
    }

    @Override
    protected boolean getHasNewRobotPose() {
        return photonPipelineResult != null && photonPipelineResult.hasTargets() && photonEstimatorResult.isPresent();
    }

    @Override
    protected int getVisibleTargetCount() {
        return photonPipelineResult != null ? photonPipelineResult.targets.size() : 0;
    }

    @Override
    protected Transform3d[] getTargetsPosesInRobotSpace(){
        if(photonPipelineResult == null) {
            return new Transform3d[]{};
        }
        List<PhotonTrackedTarget> fiducials = photonPipelineResult.getTargets();
        Transform3d[] targetsArr = new Transform3d[fiducials.size()];
        for(int i = 0; i < fiducials.size(); i++){
            targetsArr[i] = fiducials.get(i).getBestCameraToTarget();
        }

        return targetsArr;
    }
}

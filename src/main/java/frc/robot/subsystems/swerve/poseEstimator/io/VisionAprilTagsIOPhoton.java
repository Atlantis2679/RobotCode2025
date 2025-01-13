package frc.robot.subsystems.swerve.poseEstimator.io;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.swerve.poseEstimator.PoseEstimatorConstants;

public class VisionAprilTagsIOPhoton extends VisionAprilTagsIO {
    private final PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera camera;
    private List<PhotonPipelineResult> photonPipelineResults;
    private List<EstimatedRobotPose> photonEstimatorResults;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, String cameraName, AprilTagFieldLayout tagLayout) {
        super(fieldsTable);

        camera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                PoseEstimatorConstants.ROBOT_TO_CAMERA_TRANSFORM_PHOTON_FRONT);
    }
    private int count = 0;
    @Override
    public void periodicBeforeFields() {
        if(camera.getAllUnreadResults().size() > 0) {
            count++;
        }
        // photonPipelineResults = camera.getAllUnreadResults();
        // photonEstimatorResults = new ArrayList<>();
        // for(PhotonPipelineResult pipelineResult : photonPipelineResults) {
        //     Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(pipelineResult);
        //     if(pose.isPresent()){
        //         photonEstimatorResults.add(pose.get());
        //     }
        // }
    }

    @Override
    protected double[] getCameraTimestampsSeconds() {
        return new double[] {count};
        // double[] timestamps = new double[photonEstimatorResults.size()];
        
        // for (int i = 0; i < timestamps.length; i++) {
        //     timestamps[i] = photonEstimatorResults.get(i).timestampSeconds;
        // }
        // return timestamps;
    }

    @Override
    protected Pose3d[] getRobotPoses() {
        // photonPipelineResults = camera.getAllUnreadResults();
        // if(photonPipelineResults.size() <= 0){
            return new Pose3d[0];
        // }
        // Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(photonPipelineResults.get(0));
        // return new Pose3d[]{pose.isEmpty() ? new Pose3d() : pose.get().estimatedPose};
        // Pose3d[] robotPoses = new Pose3d[photonEstimatorResults.size()];
        
        // for (int i = 0; i < robotPoses.length; i++) {
        //     robotPoses[i] = photonEstimatorResults.get(i).estimatedPose;
        // }
        // return robotPoses; 
    }
}

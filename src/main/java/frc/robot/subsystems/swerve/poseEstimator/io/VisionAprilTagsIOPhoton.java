package frc.robot.subsystems.swerve.poseEstimator.io;

import java.util.ArrayList;
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
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera camera;
    private List<PhotonPipelineResult> photonPipelineResults;
    private List<EstimatedRobotPose> photonEstimatorResults;
    private final AprilTagFieldLayout tagLayout;

    public VisionAprilTagsIOPhoton(LogFieldsTable fieldsTable, String cameraName, AprilTagFieldLayout tagLayout,
            Transform3d robotToCameraTransform) {
        super(fieldsTable);

        this.tagLayout = tagLayout;

        camera = new PhotonCamera(cameraName);
        photonPoseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                robotToCameraTransform);
    }

    @Override
    public void periodicBeforeFields() {
        photonPipelineResults = camera.getAllUnreadResults();
        photonEstimatorResults = new ArrayList<>();
        for (PhotonPipelineResult pipelineResult : photonPipelineResults) {
            Optional<EstimatedRobotPose> pose = photonPoseEstimator.update(pipelineResult);
            if (pose.isPresent()) {
                photonEstimatorResults.add(pose.get());
            }
        }
    }

    @Override
    protected double[] getCameraTimestampsSeconds() {
        double[] timestamps = new double[photonEstimatorResults.size()];

        for (int i = 0; i < timestamps.length; i++) {
            timestamps[i] = photonEstimatorResults.get(i).timestampSeconds;
        }
        return timestamps;
    }

    @Override
    protected Pose3d[] getRobotPoses() {
        Pose3d[] robotPoses = new Pose3d[photonEstimatorResults.size()];

        for (int i = 0; i < robotPoses.length; i++) {
            robotPoses[i] = photonEstimatorResults.get(i).estimatedPose;
        }
        return robotPoses;
    }

    @Override
    protected Pose3d[][] getTagsPoses() {
        Pose3d[][] tagsPoses = new Pose3d[photonEstimatorResults.size()][];
        for (int i = 0; i < photonEstimatorResults.size(); i++) {
            List<PhotonTrackedTarget> targets = photonEstimatorResults.get(i).targetsUsed;
            tagsPoses[i] = new Pose3d[targets.size()];
            for (int j = 0; j < targets.size(); j++) {
                tagsPoses[i][j] = tagLayout.getTagPose(targets.get(j).fiducialId).orElse(new Pose3d());
            }
        }
        return tagsPoses;
    }

    @Override
    protected double[][] getTagsAmbiguities() {
        double[][] ambiguitys = new double[photonEstimatorResults.size()][];
        for (int i = 0; i < photonEstimatorResults.size(); i++) {
            List<PhotonTrackedTarget> targets = photonEstimatorResults.get(i).targetsUsed;
            ambiguitys[i] = new double[targets.size()];
            for (int j = 0; j < targets.size(); j++) {
                ambiguitys[i][j] = targets.get(j).getPoseAmbiguity();
            }
        }
        return ambiguitys;
    }

    @Override
    protected boolean getIsCameraConnected() {
        return camera.isConnected();
    }
}

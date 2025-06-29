package frc.robot.subsystems.swerve.poseEstimator.io;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import atlantis2679.lib.logfields.IOBase;
import atlantis2679.lib.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public final Supplier<Pose3d[]> posesEstimates = fields.addObjectArray("poseEstimates", this::getRobotPoses, new Pose3d[0]);
    public final Supplier<double[]> cameraTimestampsSeconds = fields.addDoubleArray("cameraTimestampsSeconds",
            this::getCameraTimestampsSeconds);
    public final Supplier<Pose3d[][]> tagsPoses = fields.addObjectMatrix("tagsPoses", this::getTagsPoses, new Pose3d[0][0]);
    public final Supplier<double[][]> tagsAmbiguities = fields.addDoubleMatrix("tagsAmbiguities", this::getTagsAmbiguities);
    public final BooleanSupplier isCameraConnected = fields.addBoolean("isCameraConnected", this::getIsCameraConnected);

    protected VisionAprilTagsIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    public abstract Transform3d getRobotToCameraTransform();

    protected abstract double[] getCameraTimestampsSeconds();

    protected abstract Pose3d[] getRobotPoses();

    protected abstract Pose3d[][] getTagsPoses();

    protected abstract double[][] getTagsAmbiguities();

    protected abstract boolean getIsCameraConnected();

    public abstract Transform3d getCameraTransform();
}

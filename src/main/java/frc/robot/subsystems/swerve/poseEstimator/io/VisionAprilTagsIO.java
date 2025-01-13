package frc.robot.subsystems.swerve.poseEstimator.io;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import frc.lib.logfields.IOBase;
import frc.lib.logfields.LogFieldsTable;

public abstract class VisionAprilTagsIO extends IOBase {
    public final Supplier<Pose3d[]> poseEstimate = fields.addObjectArray("poseEstimate", this::getRobotPoses, new Pose3d[0]);
    public final Supplier<double[]> cameraTimestampSeconds = fields.addDoubleArray("cameraTimestampSeconds",
            this::getCameraTimestampsSeconds);

    protected VisionAprilTagsIO(LogFieldsTable fieldsTable) {
        super(fieldsTable);
    }

    protected abstract double[] getCameraTimestampsSeconds();

    protected abstract Pose3d[] getRobotPoses();
}

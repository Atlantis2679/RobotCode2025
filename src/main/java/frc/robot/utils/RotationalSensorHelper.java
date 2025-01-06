package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;

public class RotationalSensorHelper {
    private Rotation2d measuredAngle;
    private Rotation2d offset;

    public RotationalSensorHelper(Rotation2d initialMeasuredAngle, Rotation2d initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
    }

    public RotationalSensorHelper(Rotation2d initalMeasuredAngle) {
        this(initalMeasuredAngle, new Rotation2d(0));
    }

    public void update(Rotation2d measuredAngle) {
        this.measuredAngle = measuredAngle;
    }

    public Rotation2d getMeasuredAngle() {
        return measuredAngle;
    }

    public Rotation2d getAngle() {
        return measuredAngle.minus(offset);
    }

    public void resetAngle(Rotation2d newAngle) {
        offset = measuredAngle.minus(newAngle);
    }

    public void setOffset(Rotation2d offset) {
        this.offset = offset;
    }

    public Rotation2d getOffset() {
        return offset;
    }
}
package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.valueholders.DoubleHolder;

public class PrimitiveRotationalSensorHelper implements Tuneable {
    private double measuredAngle;
    private double offset;
    private double previousAngle;
    private double currTimeSec;
    private double prevTimeSec;
    private double calculatedAngle;

    private boolean continousWrapEnabled;
    private double continousWrapUpperBound;
    private double continousWrapLowerBound;
    private double fullRotation;

    public PrimitiveRotationalSensorHelper(double initialMeasuredAngle, double initialOffset) {
        measuredAngle = initialMeasuredAngle;
        offset = initialOffset;
        prevTimeSec = Timer.getFPGATimestamp();
    }

    public PrimitiveRotationalSensorHelper(double initalMeasuredAngle) {
        this(initalMeasuredAngle, 0);
    }

    public void update(double measuredAngle) {
        this.measuredAngle = measuredAngle;
        prevTimeSec = currTimeSec;
        currTimeSec = Timer.getFPGATimestamp();
        previousAngle = calculatedAngle;
        recalculateAngle();
    }

    public double getMeasuredAngle() {
        return measuredAngle;
    }

    public double getVelocity() {
        double deltaTime = currTimeSec - prevTimeSec;
        if (deltaTime == 0) {
            DriverStation.reportWarning(
                    "You should not request velocity after no time passed (probably called in initial loop).", true);
            return 0;
        }
        return (getAngle() - previousAngle) / deltaTime;
    }

    public void recalculateAngle() {
        calculatedAngle = measuredAngle - offset;
        if (continousWrapEnabled) {
            while (calculatedAngle > continousWrapUpperBound) {
                calculatedAngle -= fullRotation;
            }
            while (calculatedAngle < continousWrapLowerBound) {
                calculatedAngle += fullRotation;
            }
        }
    }

    public double getAngle() {
        return calculatedAngle;
    }

    public void resetAngle(double newAngle) {
        setOffset(measuredAngle - newAngle);
    }

    public void enableContinousWrap(double upperBound, double fullRotation) {
        continousWrapEnabled = true;
        this.fullRotation = fullRotation;
        continousWrapUpperBound = upperBound;
        continousWrapLowerBound = upperBound - fullRotation;
        recalculateAngle();
    }

    public void setOffset(double offset) {
        this.offset = offset;
        recalculateAngle();
    }

    public double getOffset() {
        return offset;
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        DoubleHolder angleToResetDegrees = new DoubleHolder(0);
        builder.addDoubleProperty("raw angle measurment",
                this::getMeasuredAngle, null);

        builder.addDoubleProperty("calculated angle", this::getAngle, null);

        builder.addDoubleProperty("offset", this::getOffset,
                this::setOffset);

        builder.addDoubleProperty("angle to reset", angleToResetDegrees::get,
                angleToResetDegrees::set);

        builder.addChild("reset!", new InstantCommand(() -> {
            resetAngle(angleToResetDegrees.get());
        }).ignoringDisable(true));
    }
}
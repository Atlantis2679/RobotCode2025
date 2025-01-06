package frc.robot.utils;

import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Extension of the standard CommandXboxController with the following features:
 * - Lossless automatic deadband handling, preserving values below the deadband.
 * - Inverted Y axis for a more intuitive up-positive orientation.
 * - Squared axis values getters, providing finer control sensitivity.
 */
public class NaturalXboxController extends CommandXboxController {
    private double deadband;

    public NaturalXboxController(int port, double deadband) {
        super(port);
        this.deadband = deadband;
    }

    public NaturalXboxController(int port) {
        this(port, 0.05);
    }

    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    @Override
    public double getRightX() {
        return applyDeadband(super.getRightX());
    }

    @Override
    public double getRightY() {
        return -1 * applyDeadband(super.getRightY());
    }

    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return -1 * applyDeadband(super.getLeftY());
    }

    @Override
    public double getLeftTriggerAxis() {
        return applyDeadband(super.getLeftTriggerAxis());
    }

    @Override
    public double getRightTriggerAxis() {
        return applyDeadband(super.getRightTriggerAxis());
    }

    public double getSquaredRightX() {
        return square(getRightX());
    }

    public double getSquaredRightY() {
        return square(getRightY());
    }

    public double getSquaredLeftX() {
        return square(getLeftX());
    }

    public double getSquaredLeftY() {
        return square(getLeftY());
    }

    public double getSquaredLeftTriggerAxis() {
        return square(getLeftTriggerAxis());
    }

    public double getSquaredRightTriggerAxis() {
        return square(getRightTriggerAxis());
    }

    /**
     * Applies a deadband by considering values within the deadband as the new zero.
     * Values outside the deadband are adjusted to be relative to the new range.
     *
     * @param value    The input value to apply the deadband to.
     * @param deadband The deadband range. Values within this range are considered
     *                 the new zero.
     * @return The adjusted value after applying the deadband.
     */
    public double applyDeadband(double value) {
        return Math.abs(value) < deadband
                ? 0
                : (value - (Math.signum(value) * deadband)) / (1 - deadband);
    }

    public double square(double value) {
        return Math.pow(value, 2) * (value >= 0 ? 1 : -1);
    }

    public Trigger axisLessThan(int axis, double threshold) {
        return axisLessThan(axis, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    public Trigger axisLessThanLeftX(double threshold) {
        return axisLessThan(Axis.kLeftX.value, threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }
}
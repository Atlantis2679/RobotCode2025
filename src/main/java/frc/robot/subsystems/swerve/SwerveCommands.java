package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.tuneables.extensions.TuneableWrapperCommand;
import frc.lib.valueholders.BooleanHolder;
import frc.robot.subsystems.swerve.commands.SwerveDriverController;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TuneableCommand driverController(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier, BooleanSupplier isSensetiveMode) {

        return new SwerveDriverController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
                isFieldRelativeSupplier, isSensetiveMode);
    }

    // mostly for checking max module speed
    public Command driveForwardVoltage(DoubleSupplier forwardPrecentageSupplier) {
        return swerve.run(() -> swerve.drive(forwardPrecentageSupplier.getAsDouble() * MAX_VOLTAGE, 0, 0, false, true));
    }

    public TuneableCommand rotateToAngle(DoubleSupplier targetAngleDegreesCCW) {
        PIDController pidController = new PIDController(RotateToAngle.KP, RotateToAngle.KI, RotateToAngle.KD);
        pidController.enableContinuousInput(-180, 180);
        return TuneableCommand.wrap(swerve.runOnce(() -> {
            pidController.reset();
        }).andThen(swerve.run(() -> {
            swerve.drive(0, 0, pidController.calculate(swerve.getYawDegreesCCW().getDegrees(),
                    targetAngleDegreesCCW.getAsDouble()), false, true);
        })), (builder) -> {
            builder.addChild("PIDController", pidController);
        });
    }

    public TuneableCommand controlModules(DoubleSupplier turnXSupplier, DoubleSupplier turnYSupplier,
            DoubleSupplier speedSupplier) {
        return TuneableCommand.wrap(tuneableBuilder -> {
            BooleanHolder optimizeState = tuneableBuilder.addBoolean("optimize state", true);
            return new RunCommand(() -> {
                double turnY = turnYSupplier.getAsDouble();
                double turnX = turnXSupplier.getAsDouble();
                double speed = speedSupplier.getAsDouble();

                SwerveModuleState[] moduleStates = new SwerveModuleState[4];
                for (int i = 0; i < moduleStates.length; i++) {

                    double turnAngleRadians = turnX != 0 || turnY != 0
                            ? Math.atan2(turnY, turnX) - Math.toRadians(90)
                            : 0;

                    moduleStates[i] = new SwerveModuleState(
                            speed * SwerveContants.MAX_MODULE_VELOCITY_MPS,
                            new Rotation2d(turnAngleRadians));
                }
                swerve.setModulesState(moduleStates, false, optimizeState.get(), false);
            }, swerve);
        });
    }

    public Command xWheelLock() {
        return swerve.runOnce(() -> {
            SwerveModuleState[] moduleStates = new SwerveModuleState[4];
            moduleStates[0] = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
            moduleStates[1] = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
            moduleStates[2] = new SwerveModuleState(0, Rotation2d.fromDegrees(135));
            moduleStates[3] = new SwerveModuleState(0, Rotation2d.fromDegrees(-135));

            swerve.setModulesState(moduleStates, false, true, false);
        });
    }

    public Command driveToPosePID(Supplier<Pose2d> targetPoseSupplier) {
        PIDController xController = new PIDController(DriveToPose.X_KP, DriveToPose.X_KI,
                DriveToPose.X_KD);
        PIDController yController = new PIDController(DriveToPose.Y_KP, DriveToPose.Y_KI,
                DriveToPose.Y_KD);
        PIDController thetaController = new PIDController(DriveToPose.ANGLE_KP,
                DriveToPose.ANGLE_KI, DriveToPose.ANGLE_KD);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        TuneablesManager.add("Swerve/driveToPoseWithPID/xController", xController);
        TuneablesManager.add("Swerve/driveToPoseWithPID/yController", yController);
        TuneablesManager.add("Swerve/driveToPoseWithPID/thetaController", thetaController);
        return TuneableWrapperCommand.wrap(swerve.run(() -> {
            Pose2d currentPose = swerve.getPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            double thetaSpeed = thetaController.calculate(
                    currentPose.getRotation().getRadians(),
                    targetPose.getRotation().getRadians());

            xSpeed = swerve.getIsRedAlliance() ? -xSpeed : xSpeed;
            ySpeed = swerve.getIsRedAlliance() ? ySpeed : -ySpeed;

            swerve.drive(xSpeed, ySpeed, -thetaSpeed, true, true);
        })
                .finallyDo((interrupted) -> {
                    xController.reset();
                    yController.reset();
                    thetaController.reset();
                    swerve.stop();
                }), (builder) -> {
                    builder.addChild("X PID", xController);
                    builder.addChild("Y PID", yController);
                    builder.addChild("Rotate PID", thetaController);
                });
    }

    public Command stop() {
        return swerve.run(swerve::stop).ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.Arrays;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import atlantis2679.lib.tunables.extensions.TunableCommand;
import atlantis2679.lib.tunables.extensions.TunableWrapperCommand;
import atlantis2679.lib.valueholders.BooleanHolder;
import atlantis2679.lib.valueholders.ValueHolder;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.commands.SwerveDriverController;

public class SwerveCommands {
    private final Swerve swerve;

    public SwerveCommands(Swerve swerve) {
        this.swerve = swerve;
    }

    public TunableCommand driverController(DoubleSupplier forwardSupplier, DoubleSupplier sidewaysSupplier,
            DoubleSupplier rotationSupplier, BooleanSupplier isFieldRelativeSupplier, BooleanSupplier isSensetiveMode) {

        return new SwerveDriverController(swerve, forwardSupplier, sidewaysSupplier, rotationSupplier,
                isFieldRelativeSupplier, isSensetiveMode);
    }

    // mostly for checking max module speed
    public Command driveForwardVoltage(DoubleSupplier forwardPrecentageSupplier) {
        return swerve.run(
                () -> swerve.drive(forwardPrecentageSupplier.getAsDouble() * MAX_VOLTAGE, 0, 0, false, true, true));
    }

    public TunableCommand rotateToAngle(DoubleSupplier targetAngleDegreesCCW) {
        PIDController pidController = new PIDController(RotateToAngle.KP, RotateToAngle.KI, RotateToAngle.KD);
        pidController.enableContinuousInput(-180, 180);
        return TunableCommand.wrap(swerve.runOnce(() -> {
            pidController.reset();
        }).andThen(swerve.run(() -> {
            swerve.drive(0, 0, pidController.calculate(swerve.getYawCCW().getDegrees(),
                    targetAngleDegreesCCW.getAsDouble()), false, true, true);
        })), (builder) -> {
            builder.addChild("PIDController", pidController);
        });
    }

    public TunableCommand controlModules(DoubleSupplier turnXSupplier, DoubleSupplier turnYSupplier,
            DoubleSupplier speedSupplier) {
        return TunableCommand.wrap(tuneableBuilder -> {
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

    public TunableCommand driveToPosePID(Supplier<Pose2d> targetPoseSupplier) {
        PIDController xController = new PIDController(DriveToPose.X_KP, DriveToPose.X_KI,
                DriveToPose.X_KD);
        PIDController yController = new PIDController(DriveToPose.Y_KP, DriveToPose.Y_KI,
                DriveToPose.Y_KD);
        PIDController thetaController = new PIDController(DriveToPose.ANGLE_KP,
                DriveToPose.ANGLE_KI, DriveToPose.ANGLE_KD);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        return TunableWrapperCommand.wrap(swerve.run(() -> {
            Pose2d currentPose = swerve.getPose();
            Pose2d targetPose = targetPoseSupplier.get();

            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            double thetaSpeed = thetaController.calculate(
                    swerve.getPose().getRotation().getRadians(),
                    targetPose.getRotation().getRadians());

            xSpeed = MathUtil.clamp(swerve.getIsRedAlliance() ? -xSpeed : xSpeed, -2, 2);
            ySpeed = MathUtil.clamp(swerve.getIsRedAlliance() ? ySpeed : -ySpeed, -2, 2);
            thetaSpeed = MathUtil.clamp(-thetaSpeed, -3, 3);

            Logger.recordOutput("Swerve/Commands/desired pose", targetPose);
            swerve.drive(xSpeed, ySpeed, thetaSpeed, true, true, false);
        })
        .finallyDo((interrupted) -> {
            xController.reset();
            yController.reset();
            thetaController.reset();
            swerve.stop();
        }).withName("driveToPosePID"), (builder) -> {
            builder.addChild("X PID", xController);
            builder.addChild("Y PID", yController);
            builder.addChild("Rotate PID", thetaController);
        });
    }

    private static Pose2d[] flipPosesArr(Pose2d[] poses) {
        Pose2d[] flippedPoses = new Pose2d[poses.length];
        for (int i = 0; i < poses.length; i++) {
            flippedPoses[i] = FlippingUtil.flipFieldPose(poses[i]);
        }
        return flippedPoses;
    }

    public TunableCommand alignToReef(boolean isLeftSide) {
        ValueHolder<Pose2d> desiredPose = new ValueHolder<Pose2d>(null);
        Pose2d[] reefPoses = isLeftSide ? FieldConstants.REEF_LEFT_BRANCHES_POSES
                : FieldConstants.REEF_RIGHT_BRANCHES_POSES;

        TunableCommand driveToDesiredPose = driveToPosePID(desiredPose::get);

        return TunableCommand.wrap(Commands.runOnce(() -> {
            desiredPose.set(swerve.getPose().nearest(Arrays.asList(
                    swerve.getIsRedAlliance() ? flipPosesArr(reefPoses) : reefPoses)));
        }).andThen(Commands.waitUntil(() -> desiredPose.get().getTranslation()
                .getDistance(swerve.getPose().getTranslation()) < AlignToReef.MIN_DISTANCE)
                .andThen(driveToDesiredPose.asProxy()))
                .withName("align to reef"), (builder) -> {
                    driveToDesiredPose.initTunable(builder);
                });
    }

    public Command stop() {
        return swerve.run(swerve::stop);
    }
}

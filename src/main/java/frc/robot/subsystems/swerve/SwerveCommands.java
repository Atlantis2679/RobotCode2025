package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.BooleanHolder;
import frc.robot.subsystems.swerve.SwerveContants.DriveToPose;
import frc.robot.subsystems.swerve.SwerveContants.RotateToAngle;
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
            swerve.drive(0, 0, pidController.calculate(swerve.getYawCCW().getDegrees(), targetAngleDegreesCCW.getAsDouble()), false, true);
        })), (builder) -> {
            builder.addChild("PIDController", pidController);
        });
    }

    public TuneableCommand controlModules(DoubleSupplier steerXSupplier, DoubleSupplier steerYSupplier,
            DoubleSupplier speedSupplier) {
        return TuneableCommand.wrap(tuneableBuilder -> {
            BooleanHolder optimizeState = tuneableBuilder.addBoolean("optimize state", true);
            return new RunCommand(() -> {
                double steerY = steerYSupplier.getAsDouble();
                double steerX = steerXSupplier.getAsDouble();
                double speed = speedSupplier.getAsDouble();
                Logger.recordOutput("angle", steerX != 0 || steerY != 0
                        ? Math.atan2(steerY, steerX) - Math.toRadians(90)
                        : 0);
                SwerveModuleState[] moduleStates = new SwerveModuleState[4];
                for (int i = 0; i < moduleStates.length; i++) {
                    moduleStates[i] = new SwerveModuleState(
                            speed * SwerveContants.MAX_MODULE_SPEED_MPS,
                            new Rotation2d(
                                    steerX != 0 || steerY != 0
                                            ? Math.atan2(steerY, steerX) - Math.toRadians(90)
                                            : 0));
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

            swerve.setModulesState(moduleStates, false, false, false);
        });
    }

    public Command driveToPose(Pose2d targetPoseBlueAlliance) {
        return Commands.defer(() -> {
            Pose2d targetPose = swerve.getIsRedAlliance()
                    ? GeometryUtil.flipFieldPose(targetPoseBlueAlliance)
                    : targetPoseBlueAlliance;

            PathPlannerPath path = new PathPlannerPath(
                    PathPlannerPath.bezierFromPoses(swerve.getPose(), targetPose),
                    new PathConstraints(DriveToPose.MAX_VELOCITY_MPS, DriveToPose.MAX_ACCELERATION_MPS,
                            DriveToPose.MAX_ANGULAR_VELOCITY_RPS, DriveToPose.MAX_ANGULAR_ACCELERATION_RPS),
                    new GoalEndState(DriveToPose.GOAL_VELOCITY, targetPose.getRotation(), DriveToPose.ROTATE_FAST));

            path.preventFlipping = true;
            return AutoBuilder.followPath(path);

        }, Set.of(swerve));
    }

    public Command stop() {
        return swerve.run(swerve::stop).ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

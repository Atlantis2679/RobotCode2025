package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    
    public Command driveToPose(Pose2d targetPoseBlueAlliance, Command driveCommand) {
        return Commands.sequence(
            Commands.defer(() -> {
                Pose2d targetPose = swerve.getIsRedAlliance()
                        ? FlippingUtil.flipFieldPose(targetPoseBlueAlliance)
                        : targetPoseBlueAlliance;
    
                PathPlannerPath pathToPose = new PathPlannerPath(
                    PathPlannerPath.waypointsFromPoses(swerve.getPose(), targetPose),
                    new PathConstraints(
                        DriveToPose.MAX_VELOCITY_MPS, DriveToPose.MAX_ACCELERATION_MPS,
                        DriveToPose.MAX_ANGULAR_VELOCITY_RPS, DriveToPose.MAX_ANGULAR_ACCELERATION_RPS
                    ),
                    null,
                    new GoalEndState(DriveToPose.GOAL_VELOCITY, targetPose.getRotation())
                );
    
                pathToPose.preventFlipping = true;
    
                System.out.println("[INFO] Starting driveToPose command...");
    
                return AutoBuilder.followPath(pathToPose)
                    .until(() -> {
                        double distance = swerve.getPose().getTranslation().getDistance(targetPose.getTranslation());
                        System.out.println("[DEBUG] Distance to target: " + distance);
                        return distance < 0.1; // ✅ Stop if within 10 cm
                    })
                    .withTimeout(4.0) // ⏳ Prevents infinite running
                    .andThen(() -> {
                        System.out.println("[INFO] Reached target, stopping...");
                        swerve.stop(); // ✅ STOP all motion
                    });
            }, Set.of(swerve)),
    
            // ⏹️ Ensure manual control is restored
            Commands.runOnce(() -> {
                System.out.println("[INFO] Restoring manual control...");
                swerve.setDefaultCommand(driveCommand);
            }, swerve)
        ).finallyDo((interrupted) -> {
            // 🚨 Ensure command always ends correctly
            System.out.println("[INFO] driveToPose finished. Cleaning up.");
            swerve.stop();
            swerve.setDefaultCommand(driveCommand);
        });
    }
    
    
    
    
    
    public Command stop() {
        return swerve.run(swerve::stop).ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

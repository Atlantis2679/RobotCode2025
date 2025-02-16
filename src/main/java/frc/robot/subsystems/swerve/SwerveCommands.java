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
    
    public Command driveToPoseWithPID(Pose2d targetPose, Command driveCommand) {
        PIDController xController = new PIDController(0.7, 0.0, 0.0); 
        PIDController yController = new PIDController(0.5, 0.0, 0.0);
        PIDController thetaController = new PIDController(0.1, 0.0, 0.01); // Lower P, Add D
    
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 
    
        return Commands.run(() -> {
            // Get current robot pose
            Pose2d currentPose = swerve.getPose();
    
            // Calculate movement corrections
            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
    
            // ✅ Fix: Use proper angle wrapping
            double angleError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
            double thetaSpeed = thetaController.calculate(0, angleError);
    
            // ✅ Fix: Negate thetaSpeed if robot turns wrong way
            swerve.drive(xSpeed, ySpeed, -thetaSpeed, false, false);
    
            // Debugging
            System.out.println("[DEBUG] X Error: " + (targetPose.getX() - currentPose.getX()));
            System.out.println("[DEBUG] Y Error: " + (targetPose.getY() - currentPose.getY()));
            System.out.println("[DEBUG] Rotation Error: " + angleError);
    
        }, swerve)
        .until(() -> {
            double angleError = targetPose.getRotation().minus(swerve.getPose().getRotation()).getRadians();
            return Math.abs(targetPose.getX() - swerve.getPose().getX()) < 0.1 &&
                   Math.abs(targetPose.getY() - swerve.getPose().getY()) < 0.1 &&
                   Math.abs(angleError) < Math.toRadians(2.0); // ✅ Fix: Use angleError instead of raw subtraction
        })
        .finallyDo((interrupted) -> {
            System.out.println("[INFO] Reached target! Stopping...");
            swerve.stop();
            swerve.setDefaultCommand(driveCommand);
        });
    }
    
    
    
    
    
    
    public Command stop() {
        return swerve.run(swerve::stop).ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

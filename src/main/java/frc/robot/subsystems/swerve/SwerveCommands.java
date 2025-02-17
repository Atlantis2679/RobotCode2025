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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.BooleanHolder;
import frc.robot.RobotContainer;
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
        PIDController xController = new PIDController(2.1, 0.0, 0.0); 
        PIDController yController = new PIDController(1.7, 0.0, 0.0);
        PIDController thetaController = new PIDController(1.1, 0.0, 0.01); 
        
        thetaController.enableContinuousInput(-Math.PI, Math.PI); 
        
        return Commands.run(() -> {
            Pose2d currentPose = swerve.getPose();

            double xSpeed = xController.calculate(currentPose.getX(), targetPose.getX());
            double ySpeed = yController.calculate(currentPose.getY(), targetPose.getY());
            
            double angleError = targetPose.getRotation().minus(currentPose.getRotation()).getRadians();
            double thetaSpeed = thetaController.calculate(0, angleError);
            
            int direction = swerve.getIsRedAlliance() ? -1 : 1;

            swerve.drive(xSpeed * direction, ySpeed * -direction, -thetaSpeed * direction, false, false);
        
        }, swerve)
        .until(() -> {
            ChassisSpeeds chassisSpeeds = swerve.getRobotRelativeChassisSpeeds();
            
            double currentXVelocity = chassisSpeeds.vxMetersPerSecond;
            double currentYVelocity = chassisSpeeds.vyMetersPerSecond;
            double currentAngularVelocity = chassisSpeeds.omegaRadiansPerSecond;
            
            boolean atXPosition = swerve.atTranslationPosition(swerve.getPose().getX(), targetPose.getX(), currentXVelocity);
            boolean atYPosition = swerve.atTranslationPosition(swerve.getPose().getY(), targetPose.getY(), currentYVelocity);
            boolean atRotation = swerve.atAngle(targetPose.getRotation());
            return atXPosition && atYPosition && atRotation;
        })
        .finallyDo((interrupted) -> {
            swerve.stop();
            swerve.setDefaultCommand(driveCommand);
        });
    }
    
    public Command stop() {
        return swerve.run(swerve::stop).ignoringDisable(true)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}

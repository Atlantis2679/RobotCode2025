package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveContants.PathPlanner;
import frc.robot.subsystems.swerve.io.GyroIO;
import frc.robot.subsystems.swerve.io.GyroIONavX;
import frc.robot.subsystems.swerve.io.GyroIOSim;
import frc.robot.subsystems.swerve.poseEstimator.PoseEstimatorWithVision;
import frc.robot.utils.BuiltInAccelerometerLogged;
import frc.robot.utils.LocalADStarAK;
import frc.robot.utils.RotationalSensorHelper;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.RobotMap.CANBUS.*;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase implements Tuneable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final GyroIO gyroIO = Robot.isSimulation()
            ? new GyroIOSim(fieldsTable.getSubTable("Gyro"))
            : new GyroIONavX(fieldsTable.getSubTable("Gyro"));

    @SuppressWarnings("unused")
    private final BuiltInAccelerometerLogged builtInAccelerometer = new BuiltInAccelerometerLogged(
            fieldsTable.getSubTable("RoboRio Accelerometer"));

    // Should be FL, FR, BL, BR
    private final SwerveModule[] modules = {
            new SwerveModule(0, ModuleFL.DRIVE_MOTOR_ID, ModuleFL.ANGLE_MOTOR_ID, ModuleFL.ENCODER_ID,
                    MODULE_FL_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(1, ModuleFR.DRIVE_MOTOR_ID, ModuleFR.ANGLE_MOTOR_ID, ModuleFR.ENCODER_ID,
                    MODULE_FR_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(2, ModuleBL.DRIVE_MOTOR_ID, ModuleBL.ANGLE_MOTOR_ID, ModuleBL.ENCODER_ID,
                    MODULE_BL_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(3, ModuleBR.DRIVE_MOTOR_ID, ModuleBR.ANGLE_MOTOR_ID, ModuleBR.ENCODER_ID,
                    MODULE_BR_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable)
    };

    // The x and y might seem a bit weird, but this is how they are defined in
    // WPILib. For more info:
    // https://docs.wpilib.org/he/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
    public final Translation2d FRONT_LEFT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_M / 2,
            -SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_LEFT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_LENGTH_M / 2,
            SwerveContants.TRACK_WIDTH_M / 2);
    public final Translation2d BACK_RIGHT_LOCATION = new Translation2d(
            -SwerveContants.TRACK_WIDTH_M / 2,
            -SwerveContants.TRACK_LENGTH_M / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            FRONT_LEFT_LOCATION,
            FRONT_RIGHT_LOCATION,
            BACK_LEFT_LOCATION,
            BACK_RIGHT_LOCATION);

    private RotationalSensorHelper gyroYawHelperCCW;

    private final List<BiConsumer<Pose2d, Boolean>> callbacksOnPoseUpdate = new ArrayList<>();
    private final PoseEstimatorWithVision poseEstimator;

    private final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

    public Swerve() {
        fieldsTable.update();

        isRedAlliance.addDefaultOption("blue", false);
        isRedAlliance.addOption("red", true);

        gyroYawHelperCCW = new RotationalSensorHelper(
                Rotation2d.fromDegrees(gyroIO.isConnected.getAsBoolean() ? -gyroIO.yawDegreesCW.getAsDouble() : 0));

        poseEstimator = new PoseEstimatorWithVision(fieldsTable.getSubTable("poseEstimator"), getYawCCW(), getModulesPositions(), swerveKinematics);

        TuneablesManager.add("Swerve", (Tuneable) this);

        resetYaw();

        queueResetModulesToAbsolute();

        Pathfinding.setPathfinder(new LocalADStarAK());

        HolonomicPathFollowerConfig pathFollowerConfigs = new HolonomicPathFollowerConfig(
                new PIDConstants(PathPlanner.TRANSLATION_KP, PathPlanner.TRANSLATION_KI, PathPlanner.TRANSLATION_KD),
                new PIDConstants(PathPlanner.ROTATION_KP, PathPlanner.ROTATION_KI, PathPlanner.ROTATION_KD),
                MAX_MODULE_SPEED_MPS,
                TRACK_RADIUS_M,
                new ReplanningConfig());

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveVoltageChassisSpeed,
                pathFollowerConfigs,
                this::getIsRedAlliance,
                this);

        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    fieldsTable.recordOutput(
                            "PathPlanner/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    fieldsTable.recordOutput("PathPlanner/TrajectorySetpoint", targetPose);
                });
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (gyroIO.isConnected.getAsBoolean()) {
            gyroYawHelperCCW.update(Rotation2d.fromDegrees(-gyroIO.yawDegreesCW.getAsDouble()));
        } else {
            Twist2d twist = swerveKinematics.toTwist2d(
                    modules[0].getModulePositionDelta(),
                    modules[1].getModulePositionDelta(),
                    modules[2].getModulePositionDelta(),
                    modules[3].getModulePositionDelta());

            gyroYawHelperCCW.update(gyroYawHelperCCW.getMeasuredAngle().plus(Rotation2d.fromRadians(twist.dtheta)));
        }

        poseEstimator.update(gyroYawHelperCCW.getMeasuredAngle(), getModulesPositions());
        callbacksOnPoseUpdate.forEach(callback -> {
            callback.accept(getPose(), getIsRedAlliance());
        });

        fieldsTable.recordOutput("Estimated Robot Pose", getPose());
        fieldsTable.recordOutput("Module States",
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());

        fieldsTable.recordOutput("Module States Integreated",
                modules[0].getModuleStateIntegreated(),
                modules[1].getModuleStateIntegreated(),
                modules[2].getModuleStateIntegreated(),
                modules[3].getModuleStateIntegreated());

        fieldsTable.recordOutput("Robot Yaw Radians CCW", getYawCCW().getRadians());
        fieldsTable.recordOutput("Yaw Degrees CW", -getYawCCW().getDegrees());
        SmartDashboard.putBoolean("isRedAlliance", getIsRedAlliance());
        fieldsTable.recordOutput("is red alliance", getIsRedAlliance());
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : null);
        fieldsTable.recordOutput("is moving", gyroIO.isMoving.getAsBoolean());

        if (gyroIO.isMoving.getAsBoolean() || !gyroIO.isConnected.getAsBoolean()) {
            poseEstimator.update(gyroYawHelperCCW.getMeasuredAngle(), getModulesPositions());
            callbacksOnPoseUpdate.forEach(callback -> {
                callback.accept(getPose(), getIsRedAlliance());
            });
        }
    }

    public void drive(double forward, double sidewaysRightPositive, double angularVelocityCW, boolean isFieldRelative, boolean useVoltage) {
        ChassisSpeeds desiredChassisSpeeds;

        double angularVelocityCCW = -angularVelocityCW;
        double sidewaysLeftPositive = -sidewaysRightPositive;

        if (isFieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    getIsRedAlliance() ? -forward : forward,
                    getIsRedAlliance() ? -sidewaysLeftPositive : sidewaysLeftPositive,
                    angularVelocityCCW,
                    getYawCCW());
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(
                    forward,
                    sidewaysLeftPositive,
                    angularVelocityCCW);
        }

        driveChassisSpeed(desiredChassisSpeeds, useVoltage);
    }

    public void stop() {
        drive(0, 0, 0, false, false);
    }

    public void driveChassisSpeed(ChassisSpeeds speeds, boolean useVoltage) {
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.MAX_MODULE_SPEED_MPS);

        setModulesState(swerveModuleStates, true, true, useVoltage);
    }

    public void driveVoltageChassisSpeed(ChassisSpeeds speeds) {
        driveChassisSpeed(speeds, true);
    }

    public void setModulesState(SwerveModuleState[] moduleStates, boolean preventJittering, boolean optimizeState,
            boolean useVoltage) {
        fieldsTable.recordOutput("Module Desired States", moduleStates);

        for (SwerveModule module : modules) {
            module.setDesiredState(moduleStates[module.getModuleNumber()], preventJittering, optimizeState, useVoltage);
        }
    }

    public Rotation2d getYawCCW() {
        return gyroYawHelperCCW.getAngle();
    }

    public void setYawDegreesCW(double newYawDegreesCW) {
        Pose2d currentPose = getPose();

        resetPose(new Pose2d(
                currentPose.getX(),
                currentPose.getY(),
                Rotation2d.fromDegrees(-newYawDegreesCW)));
    }

    public void resetYaw() {
        setYawDegreesCW(getIsRedAlliance() ? 180 : 0);
    }

    public Pose2d getPose() {
        return poseEstimator.getPose();
    }

    public void queueResetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.queueResetToAbsolute();
        }
    }

    public void registerCallbackOnPoseUpdate(BiConsumer<Pose2d, Boolean> callback) {
        callbacksOnPoseUpdate.add(callback);
        callback.accept(getPose(), getIsRedAlliance());
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : modules) {
            modulePosition[module.getModuleNumber()] = new SwerveModulePosition(
                    module.getDriveDistanceMeters(),
                    new Rotation2d(Math.toRadians(module.getAbsoluteAngleDegrees())));
        }

        return modulePosition;
    }

    public void resetPose(Pose2d pose2d) {
        gyroYawHelperCCW.resetAngle(pose2d.getRotation());
        poseEstimator.resetPosition(gyroYawHelperCCW.getMeasuredAngle(), getModulesPositions(), pose2d);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveKinematics.toChassisSpeeds(
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());
    }

    public boolean getIsRedAlliance() {
        return isRedAlliance.get() != null && isRedAlliance.get().booleanValue();
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Swerve Subsystem", (Sendable) this);

        builder.addChild("Modules Angle PID", (Tuneable) (builderPID) -> {
            builderPID.setSendableType(SendableType.PIDController);
            builderPID.addDoubleProperty("p", modules[0]::getP, (p) -> {
                for (SwerveModule module : modules) {
                    module.setP(p);
                }
            });

            builderPID.addDoubleProperty("i", modules[0]::getI, (i) -> {
                for (SwerveModule module : modules) {
                    module.setI(i);
                }
            });

            builderPID.addDoubleProperty("d", modules[0]::getD, (d) -> {
                for (SwerveModule module : modules) {
                    module.setD(d);
                }
            });
            builderPID.addDoubleProperty("setpoint", () -> 0, null);
        });
        builder.addChild("Module 0", modules[0]);
        builder.addChild("Module 1", modules[1]);
        builder.addChild("Module 2", modules[2]);
        builder.addChild("Module 3", modules[3]);

        builder.addChild("reset modules absolute position", (Tuneable) (resetModulesBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            resetModulesBuilder.addDoubleProperty("angle to reset degrees", angleToResetDegrees::get,
                    angleToResetDegrees::set);
            resetModulesBuilder.addChild("reset!", new InstantCommand(() -> {
                for (SwerveModule swerveModule : modules) {
                    swerveModule.setAbsoluteEncoderAngleDegrees(angleToResetDegrees.get());
                }
            }).ignoringDisable(true));
        });

        builder.addChild("coast mode", run(() -> {
            for (SwerveModule module : modules) {
                module.enableCoastMode();
            }
        }).ignoringDisable(true));

        builder.addChild("reset to absolute", new InstantCommand(this::queueResetModulesToAbsolute));
    }
}

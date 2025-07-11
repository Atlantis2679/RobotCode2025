package frc.robot.subsystems.swerve;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
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
import team2679.atlantiskit.helpers.RotationalSensorHelper;
import team2679.atlantiskit.logfields.LogFieldsTable;
import team2679.atlantiskit.periodicalerts.PeriodicAlertsGroup;
import team2679.atlantiskit.tunables.SendableType;
import team2679.atlantiskit.tunables.Tunable;
import team2679.atlantiskit.tunables.TunableBuilder;
import team2679.atlantiskit.tunables.TunablesManager;
import team2679.atlantiskit.valueholders.DoubleHolder;
import frc.robot.Robot;
import frc.robot.RobotMap.CANBUS.*;

import static frc.robot.subsystems.swerve.SwerveContants.*;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BiConsumer;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

public class Swerve extends SubsystemBase implements Tunable {
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());

    private final GyroIO gyroIO = Robot.isSimulation()
            ? new GyroIOSim(fieldsTable.getSubTable("Gyro"))
            : new GyroIONavX(fieldsTable.getSubTable("Gyro"));

    @SuppressWarnings("unused")
    private final BuiltInAccelerometerLogged builtInAccelerometer = new BuiltInAccelerometerLogged(
            fieldsTable.getSubTable("RoboRio Accelerometer"));

    // Should be FL, FR, BL, BR
    private final SwerveModule[] modules = {
            new SwerveModule(0, "FL", ModuleFL.DRIVE_MOTOR_ID, ModuleFL.TURN_MOTOR_ID, ModuleFL.ENCODER_ID,
                    MODULE_FL_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(1, "FR", ModuleFR.DRIVE_MOTOR_ID, ModuleFR.TURN_MOTOR_ID, ModuleFR.ENCODER_ID,
                    MODULE_FR_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(2, "BL", ModuleBL.DRIVE_MOTOR_ID, ModuleBL.TURN_MOTOR_ID, ModuleBL.ENCODER_ID,
                    MODULE_BL_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable),
            new SwerveModule(3, "BR", ModuleBR.DRIVE_MOTOR_ID, ModuleBR.TURN_MOTOR_ID, ModuleBR.ENCODER_ID,
                    MODULE_BR_ABSOLUTE_ANGLE_OFFSET_DEGREES, fieldsTable)
    };

    // The x and y might seem a bit weird, but this is how they are defined in
    // WPILib. For more info:
    // https://docs.wpilib.org/he/stable/docs/software/advanced-controls/geometry/coordinate-systems.html
    public final Translation2d FL_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_METERS / 2,
            SwerveContants.TRACK_WIDTH_METERS / 2);
    public final Translation2d FR_LOCATION = new Translation2d(
            SwerveContants.TRACK_LENGTH_METERS / 2,
            -SwerveContants.TRACK_WIDTH_METERS / 2);
    public final Translation2d BL_LOCATION = new Translation2d(
            -SwerveContants.TRACK_LENGTH_METERS / 2,
            SwerveContants.TRACK_WIDTH_METERS / 2);
    public final Translation2d BR_LOCATION = new Translation2d(
            -SwerveContants.TRACK_LENGTH_METERS / 2,
            -SwerveContants.TRACK_WIDTH_METERS / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            FL_LOCATION,
            FR_LOCATION,
            BL_LOCATION,
            BR_LOCATION);

    private RotationalSensorHelper gyroYawHelperDegreesCCW;

    private final List<BiConsumer<Pose2d, Boolean>> callbacksOnPoseUpdate = new ArrayList<>();
    private final PoseEstimatorWithVision poseEstimator;

    private final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

    private final Debouncer gyroConnectedDebouncer = new Debouncer(GYRO_CONNECTED_DEBAUNCER_SEC);

    public Swerve() {
        fieldsTable.update();
        queueResetModulesToAbsolute();
        isRedAlliance.addDefaultOption("blue", false);
        isRedAlliance.addOption("red", true);

        fieldsTable.recordOutput("current command",
                getCurrentCommand() == null ? "none" : getCurrentCommand().getName());

        gyroYawHelperDegreesCCW = new RotationalSensorHelper(gyroIO.isConnected.getAsBoolean() ? -gyroIO.yawDegreesCW.getAsDouble() : 0);

        poseEstimator = new PoseEstimatorWithVision(fieldsTable.getSubTable("poseEstimator"), getYawCCW(),
                getModulesPositions(), swerveKinematics);

        TunablesManager.add("Swerve", (Tunable) this);

        PeriodicAlertsGroup.defaultInstance.addErrorAlert(() -> "Swerve: Gyro IS Disconnected!", () -> !getGyroConnectedDebouncer());

        resetYaw();

        ModuleConfig moduleConfig = new ModuleConfig(
                WHEEL_RADIUS_METERS,
                MAX_MODULE_VELOCITY_MPS,
                PathPlanner.FRICTION_WITH_CARPET,
                DCMotor.getFalcon500(1).withReduction(GEAR_RATIO_DRIVE),
                130,
                1);

        RobotConfig robotConfig = new RobotConfig(PathPlanner.ROBOT_MASS_KG, PathPlanner.MOMENT_OF_INERTIA,
                moduleConfig, FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);
        robotConfig.hasValidConfig();

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeChassisSpeeds,
                (speeds, feedforward) -> driveChassisSpeed(speeds, true),
                new PPHolonomicDriveController(
                        new PIDConstants(
                                PathPlanner.TRANSLATION_KP,
                                PathPlanner.TRANSLATION_KI,
                                PathPlanner.TRANSLATION_KD),
                        new PIDConstants(
                                PathPlanner.ROTATION_KP,
                                PathPlanner.ROTATION_KI,
                                PathPlanner.ROTATION_KD)),
                robotConfig,
                this::getIsRedAlliance,
                this);

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            fieldsTable.recordOutput("PathPlanner/desired pose", pose);
        });
        PathPlannerLogging.setLogCurrentPoseCallback((pose) -> {
            fieldsTable.recordOutput("PathPlanner/current pose", pose);
        });
        PathPlannerLogging.setLogActivePathCallback((path) -> {
            fieldsTable.recordOutput("PathPlanner/path", path.toArray(new Pose2d[0]));
        });
    }

    @Override
    public void periodic() {

        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (getGyroConnectedDebouncer()) {
            gyroYawHelperDegreesCCW.update(-gyroIO.yawDegreesCW.getAsDouble());
        } else {
            Twist2d twist = swerveKinematics.toTwist2d(
                    modules[0].getModulePositionDelta(),
                    modules[1].getModulePositionDelta(),
                    modules[2].getModulePositionDelta(),
                    modules[3].getModulePositionDelta());

            gyroYawHelperDegreesCCW
                    .update(gyroYawHelperDegreesCCW.getMeasuredAngle() + Math.toDegrees(twist.dtheta));
        }

        poseEstimator.update(Rotation2d.fromDegrees(gyroYawHelperDegreesCCW.getMeasuredAngle()), getModulesPositions(), getGyroConnectedDebouncer());
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
                modules[0].getModuleStateIntegrated(),
                modules[1].getModuleStateIntegrated(),
                modules[2].getModuleStateIntegrated(),
                modules[3].getModuleStateIntegrated());

        fieldsTable.recordOutput("Robot Yaw Radians CCW", getYawCCW().getRadians());
        fieldsTable.recordOutput("Yaw Degrees CW", -getYawCCW().getDegrees());
        SmartDashboard.putBoolean("isRedAlliance", getIsRedAlliance());
        fieldsTable.recordOutput("is red alliance", getIsRedAlliance());
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "none");
    }

    public void drive(double forward, double sidewaysRightPositive, double angularVelocityCW, boolean isFieldRelative,
            boolean useVoltage, boolean useGyro) {
        ChassisSpeeds desiredChassisSpeeds;

        double angularVelocityCCW = -angularVelocityCW;
        double sidewaysLeftPositive = -sidewaysRightPositive;

        if (isFieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    getIsRedAlliance() ? -forward : forward,
                    getIsRedAlliance() ? -sidewaysLeftPositive : sidewaysLeftPositive,
                    angularVelocityCCW, useGyro? getYawCCW(): getPose().getRotation());
                    fieldsTable.recordOutput("angle Pose", getPose().getRotation());
                    fieldsTable.recordOutput("angle gyro", getYawCCW());
        } else {
            desiredChassisSpeeds = new ChassisSpeeds(
                    forward,
                    sidewaysLeftPositive,
                    angularVelocityCCW);
        }

        driveChassisSpeed(desiredChassisSpeeds, useVoltage);
    }

    public void stop() {
        drive(0, 0, 0, false, false, true);
    }

    public void driveChassisSpeed(ChassisSpeeds speeds, boolean useVoltage) {
        SwerveModuleState[] swerveModuleStates = swerveKinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveContants.MAX_MODULE_VELOCITY_MPS);

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
        return Rotation2d.fromDegrees(gyroYawHelperDegreesCCW.getAngle());
    }

    public void resetYawDegreesCW(double newYawDegreesCW) {
        Pose2d currentPose = getPose();

        resetPose(new Pose2d(
                currentPose.getX(),
                currentPose.getY(),
                Rotation2d.fromDegrees(-newYawDegreesCW)));
    }

    public void resetYaw() {
        resetYawDegreesCW(getIsRedAlliance() ? 180 : 0);
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
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : modules) {
            modulePosition[module.getModuleNumber()] = module.getModulePosition();
        }

        return modulePosition;
    }

    public void resetPose(Pose2d pose2d) {
        gyroYawHelperDegreesCCW.resetAngle(pose2d.getRotation().getDegrees());
        poseEstimator.resetPosition(Rotation2d.fromDegrees(gyroYawHelperDegreesCCW.getMeasuredAngle()), getModulesPositions(), pose2d);
    }

    public ChassisSpeeds getRobotRelativeChassisSpeeds() {
        return swerveKinematics.toChassisSpeeds(
                modules[0].getModuleState(),
                modules[1].getModuleState(),
                modules[2].getModuleState(),
                modules[3].getModuleState());
    }

    public boolean getIsRedAlliance() {
        return isRedAlliance.get() != null && isRedAlliance.get().booleanValue();
    }

    public boolean getGyroConnectedDebouncer() {
        return gyroConnectedDebouncer.calculate(gyroIO.isConnected.getAsBoolean());
    }

    public void enableCoast() {
        for (SwerveModule module : modules) {
            module.enableCoastMode();
        }
    }

    @Override
    public void initTunable(TunableBuilder builder) {
        builder.addChild("Swerve Subsystem", (Sendable) this);

        builder.addChild("Modules Angle PID", (Tunable) (builderPID) -> {
            builderPID.setSendableType(SendableType.PIDController);
            builderPID.addDoubleProperty("p", modules[0]::getTurnKP, (p) -> {
                for (SwerveModule module : modules) {
                    module.setTurnKP(p);
                }
            });

            builderPID.addDoubleProperty("i", modules[0]::getTurnKI, (i) -> {
                for (SwerveModule module : modules) {
                    module.setTurnKI(i);
                }
            });

            builderPID.addDoubleProperty("d", modules[0]::getTurnKD, (d) -> {
                for (SwerveModule module : modules) {
                    module.setTurnKD(d);
                }
            });
            builderPID.addDoubleProperty("setpoint", () -> 0, null);
        });
        builder.addChild("Module 0 FL", modules[0]);
        builder.addChild("Module 1 FR", modules[1]);
        builder.addChild("Module 2 BL", modules[2]);
        builder.addChild("Module 3 BR", modules[3]);

        builder.addChild("reset modules absolute position", (Tunable) (resetModulesBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            resetModulesBuilder.addDoubleProperty("angle to reset degrees", angleToResetDegrees::get,
                    angleToResetDegrees::set);
            resetModulesBuilder.addChild("reset!", new InstantCommand(() -> {
                for (SwerveModule swerveModule : modules) {
                    swerveModule.resetAbsoluteAngleDegrees(angleToResetDegrees.get());
                }
            }).ignoringDisable(true));
        });

        builder.addChild("reset to absolute",
                new InstantCommand(this::queueResetModulesToAbsolute).ignoringDisable(true));
        
        builder.addChild("reset gyro", (Tunable) (resetGyroBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            resetGyroBuilder.addDoubleProperty("angle to reset degrees CWW", angleToResetDegrees::get,
                angleToResetDegrees::set);
            resetGyroBuilder.addChild("reset", new InstantCommand(() -> {
                resetYawDegreesCW(-angleToResetDegrees.get());
            }).ignoringDisable(true));
        });
    }
}

package frc.robot.subsystems.swerve;

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
import frc.robot.utils.RotationalSensorHelper;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.SendableType;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneableBuilder;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
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
            -SwerveContants.TRACK_WIDTH_METERS / 2,
            -SwerveContants.TRACK_LENGTH_METERS / 2);

    private final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            FL_LOCATION,
            FR_LOCATION,
            BL_LOCATION,
            BR_LOCATION);

    private RotationalSensorHelper gyroYawHelperDegreesCCW;

    private final List<BiConsumer<Pose2d, Boolean>> callbacksOnPoseUpdate = new ArrayList<>();
    private final PoseEstimatorWithVision poseEstimator;

    private final LoggedDashboardChooser<Boolean> isRedAlliance = new LoggedDashboardChooser<>("alliance");

    public Swerve() {
        fieldsTable.update();
        isRedAlliance.addDefaultOption("blue", false);
        isRedAlliance.addOption("red", true);

        gyroYawHelperDegreesCCW = new RotationalSensorHelper(
                Rotation2d.fromDegrees(gyroIO.isConnected.getAsBoolean() ? -gyroIO.yawDegreesCW.getAsDouble() : 0));

        poseEstimator = new PoseEstimatorWithVision(fieldsTable.getSubTable("poseEstimator"), getYawDegreesCCW(),
                getModulesPositions(), swerveKinematics);
                
        TuneablesManager.add("Swerve", (Tuneable) this);
    
        resetYaw();

        ModuleConfig moduleConfig = new ModuleConfig(WHEEL_RADIUS_METERS, MAX_MODULE_VELOCITY_MPS, PathPlanner.FRICTION_WITH_CARPET, DCMotor.getFalcon500(1).withReduction(GEAR_RATIO_DRIVE), MAX_VOLTAGE, 2);

        RobotConfig config = new RobotConfig(PathPlanner.ROBOT_MASS_KG, PathPlanner.MOMENT_OF_INERTIA, moduleConfig, FL_LOCATION, FR_LOCATION, BL_LOCATION, BR_LOCATION);

        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

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
            config, 
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
        // In case the modules fail to reset to absolute:
        // queueResetModulesToAbsolute();
    }

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.periodic();
        }

        if (gyroIO.isConnected.getAsBoolean()) {
            gyroYawHelperDegreesCCW.update(Rotation2d.fromDegrees(-gyroIO.yawDegreesCW.getAsDouble()));
        } else {
            Twist2d twist = swerveKinematics.toTwist2d(
                    modules[0].getModulePositionDelta(),
                    modules[1].getModulePositionDelta(),
                    modules[2].getModulePositionDelta(),
                    modules[3].getModulePositionDelta());

            gyroYawHelperDegreesCCW.update(gyroYawHelperDegreesCCW.getMeasuredAngle().plus(Rotation2d.fromRadians(twist.dtheta)));
        }

        poseEstimator.update(gyroYawHelperDegreesCCW.getMeasuredAngle(), getModulesPositions());
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

        fieldsTable.recordOutput("Robot Yaw Radians CCW", getYawDegreesCCW().getRadians());
        fieldsTable.recordOutput("Yaw Degrees CW", -getYawDegreesCCW().getDegrees());
        SmartDashboard.putBoolean("isRedAlliance", getIsRedAlliance());
        fieldsTable.recordOutput("is red alliance", getIsRedAlliance());
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : null);
    }

    public void drive(double forward, double sidewaysRightPositive, double angularVelocityCW, boolean isFieldRelative,
            boolean useVoltage) {
        ChassisSpeeds desiredChassisSpeeds;

        double angularVelocityCCW = -angularVelocityCW;
        double sidewaysLeftPositive = -sidewaysRightPositive;

        if (isFieldRelative) {
            desiredChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    getIsRedAlliance() ? -forward : forward,
                    getIsRedAlliance() ? -sidewaysLeftPositive : sidewaysLeftPositive,
                    angularVelocityCCW,
                    getYawDegreesCCW());
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

    public Rotation2d getYawDegreesCCW() {
        return gyroYawHelperDegreesCCW.getAngle();
    }
    public ChassisSpeeds getSelfRelativeVelocity() {
        return swerveKinematics.toChassisSpeeds();
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
        fieldsTable.recordOutput("poses", FieldConstants.REEF_PLACE_POSES);
        return poseEstimator.getPose();
    }


    public void queueResetModulesToAbsolute() {
        for (SwerveModule module : modules) {
            module.queueResetToAbsolute();
        }
    }

    public void registerCallbackOnPoseUpdate(BiConsumer<Pose2d, Boolean> callback) {
        callbacksOnPoseUpdate.add(callback);
        callback.accept(getPose(), getIsRedAlliance()); // Why???
    }

    public SwerveModulePosition[] getModulesPositions() {
        SwerveModulePosition[] modulePosition = new SwerveModulePosition[4];

        for (SwerveModule module : modules) {
            modulePosition[module.getModuleNumber()] = module.getModulePosition();
        }

        return modulePosition;
    }

    public void resetPose(Pose2d pose2d) {
        gyroYawHelperDegreesCCW.resetAngle(pose2d.getRotation());
        poseEstimator.resetPosition(gyroYawHelperDegreesCCW.getMeasuredAngle(), getModulesPositions(), pose2d);
    }
    public boolean atTranslationPosition(double currentPosition, double targetPosition, double currentVelocity) {
        boolean atPosition = Math.abs(currentPosition - targetPosition) < SwerveContants.TRANSLATION_TOLERANCE_METERS;
        boolean atVelocity = Math.abs(currentVelocity) < SwerveContants.TRANSLATION_VELOCITY_TOLERANCE;
        fieldsTable.recordOutput("AtTargetPosition/isStill", atVelocity);
        fieldsTable.recordOutput("atTargetPosition-" + targetPosition, atPosition);
        return atPosition && atVelocity;
    }

    public boolean atAngle(Rotation2d targetAngle) {
    double angleDifference = targetAngle.minus(getPose().getRotation()).getDegrees();
    final boolean atTargetAngle = Math.abs(angleDifference) < SwerveContants.ROTATION_TOLERANCE_DEGREES;
    double currentAngularVelocity = getRobotRelativeChassisSpeeds().omegaRadiansPerSecond;
    final boolean isAngleStill = Math.abs(currentAngularVelocity) < SwerveContants.ROTATION_VELOCITY_TOLERANCE;
    fieldsTable.recordOutput("AtTargetAngle/isStill", isAngleStill);
    fieldsTable.recordOutput("atTargetAngle-" + targetAngle, atTargetAngle);
    return atTargetAngle && isAngleStill;
    }
    public double getDistanceToPose(Pose2d targetPose) {
        double deltaX = targetPose.getX() - getPose().getX();
        double deltaY = targetPose.getY() - getPose().getY();
        double distance = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        fieldsTable.recordOutput("distanceTPose", distance);
        return distance;
    }
    public double getAngularDistance(Pose2d targetPose){
        double deltaTheta = targetPose.getRotation().minus(getPose().getRotation()).getRadians();
        double angularDistance = Math.abs(deltaTheta); 
        return angularDistance;
    }

    public Pose2d getClosestPose(Pose2d[] poses) {
        Pose2d closestPose = null;
        double minDistance = Double.MAX_VALUE;
        for (Pose2d targetPose : poses) {
            double distance = (getDistanceToPose(targetPose));
            if (distance < minDistance) {
                minDistance = distance;
                closestPose = targetPose;
            }
        }
        fieldsTable.recordOutput("closestPose", closestPose);
        return closestPose;
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

    public void enableCoast() {
        for (SwerveModule module : modules) {
            module.enableCoastMode();
        }
    }

    @Override
    public void initTuneable(TuneableBuilder builder) {
        builder.addChild("Swerve Subsystem", (Sendable) this);

        builder.addChild("Modules Angle PID", (Tuneable) (builderPID) -> {
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

        builder.addChild("reset modules absolute position", (Tuneable) (resetModulesBuilder) -> {
            DoubleHolder angleToResetDegrees = new DoubleHolder(0);
            resetModulesBuilder.addDoubleProperty("angle to reset degrees", angleToResetDegrees::get,
                    angleToResetDegrees::set);
            resetModulesBuilder.addChild("reset!", new InstantCommand(() -> {
                for (SwerveModule swerveModule : modules) {
                    swerveModule.resetAbsoluteAngleDegrees(angleToResetDegrees.get());
                }
            }).ignoringDisable(true));
        });

        builder.addChild("reset to absolute", new InstantCommand(this::queueResetModulesToAbsolute).ignoringDisable(true));
    }
}

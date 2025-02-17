package frc.robot.allcommands;

import java.util.Set;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.FieldConstants;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.allcommands.AllCommandsConstants.ManualControllers;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

public class AllCommands {
    private final Gripper gripper;
    private final Pivot pivot;
    private final Funnel funnel;
    private final Swerve swerve;

    private final GripperCommands gripperCMDs;
    private final PivotCommands pivotCMDs;
    private final FunnelCommands funnelCMDs;
    private final SwerveCommands swerveCMDs;

    public AllCommands(Gripper gripper, Pivot pivot, Funnel funnel, Swerve swerve) {
        this.gripper = gripper;
        this.pivot = pivot;
        this.funnel = funnel;
        this.swerve = swerve;

        this.gripperCMDs = new GripperCommands(gripper, funnel);
        this.pivotCMDs = new PivotCommands(pivot);
        this.funnelCMDs = new FunnelCommands(funnel);
        this.swerveCMDs = new SwerveCommands(swerve);
    }

    public Command getToPose(Pose2d targetPose2d, TuneableCommand driveCommand){
        return new DeferredCommand(() -> swerveCMDs.driveToPoseWithPID(targetPose2d, driveCommand), Set.of(swerve));
    }
    public Command intake() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE)).andThen(
            funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))).until( () -> !funnel.getIsCoralIn() && gripper.getIsCoralIn()))
            .withName("Intake");
    }


    // public static FlippablePose2d calculateTargetScoringPose() {
    //     return TARGET_SCORING_LEVEL.calculateTargetPlacingPosition(TARGET_REEF_SCORING_CLOCK_POSITION, TARGET_REEF_SCORING_SIDE);
    // }
        //     public FlippablePose2d calculateTargetPlacingPosition(FieldConstants.ReefClockPosition reefClockPosition, FieldConstants.ReefSide reefSide) {
        //     final Pose2d reefCenterPose = new Pose2d(FieldConstants.BLUE_REEF_CENTER_TRANSLATION, reefClockPosition.clockAngle);
        //     final double yTransform = reefSide.shouldFlipYTransform(reefClockPosition) ? -0.17 : 0.17;
        //     final Transform2d transform = new Transform2d(1.38, yTransform, rotationTransform);

        //     return new FlippablePose2d(reefCenterPose.plus(transform), true);
        // }
    public Command intakeStatic() {
        return funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).withName("intakeStatic");
    }

    public Command scoreL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L1))
        .andThen(gripperCMDs.score(GRIPPER_INTAKE_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
        .finallyDo(pivot::stop).withName("scoreL1");
    }

    public Command scoreStaticL1() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(gripperCMDs.score(GRIPPER_LOADING_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
            .withName("scoreStaticL1");
    }

    public Command moveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2)
        .finallyDo(() -> pivot.stop()).withName("moveToL2");
    }

    public Command moveToL3() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L3)
        .finallyDo(() -> pivot.stop()).withName("moveToL3");
    }

    public Command scoreL1Shoot() {
        return gripperCMDs.score(GRIPPER_INTAKE_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE);
    }
    public Command scoreL3(){
        return gripperCMDs.score(GRIPPER_INTAKE_VOLTAGE, GRIPPER_L3_VOLTAGE, GRIPPER_L3_VOLTAGE);
    }

    public TuneableCommand getPivotReadyAndScore() {
            return TuneableCommand.wrap((tuneableTable) -> {
            DoubleHolder angleHolder = tuneableTable.addNumber("angle", PIVOT_TUNEABLE_ANGLE);
            DoubleHolder leftGripperVoltage = tuneableTable.addNumber("upper roller speed",
                    GRIPPER_LEFT_TUNEABLE_VOLTAGE);
            DoubleHolder rightGripperVoltage = tuneableTable.addNumber("lower roller speed",
                    GRIPPER_RIGHT_TUNEABLE_VOLTAGE);
            return (pivotCMDs.moveToAngle(angleHolder.get()))
            .andThen(gripperCMDs.score(GRIPPER_INTAKE_VOLTAGE, rightGripperVoltage.get(), leftGripperVoltage.get()))
                    .withName("getPivotAngleAndScore");
        });

    }
 
    public Command manualGripperController(DoubleSupplier speed) {
        return gripperCMDs.manualController(() -> speed.getAsDouble() * ManualControllers.GRIPPER_RIGHT_SPEED_MULTIPLAYER, 
            () -> speed.getAsDouble() * ManualControllers.GRIPPER_LEFT_SPEED_MULTIPLAYER);
    }

    public Command manualFunnelController(DoubleSupplier speed) {
        return funnelCMDs.manualController(() -> speed.getAsDouble() * ManualControllers.FUNNEL_SPEED_MULTIPLAYER);
    }

    public Command manualPivotController(DoubleSupplier speed) {
        return pivotCMDs.manualController(() -> speed.getAsDouble() * ManualControllers.PIVOT_SPEED_MULTIPLAYER);
    }

    public Command stopAll() {
        return Commands.run(() -> {
            gripper.stop();
            pivot.stop();
            funnel.stop();
        }, gripper, pivot, funnel);
    }
}
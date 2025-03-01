package frc.robot.allcommands;

import java.util.Set;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.FieldConstants;
import frc.robot.allcommands.AllCommandsConstants.ManualControllers;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.leds.LedsCommands;
import frc.robot.subsystems.leds.LedsConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.subsystems.swerve.SwerveContants;

import static frc.robot.allcommands.AllCommandsConstants.*;

public class AllCommands {
    private final Gripper gripper;
    private final Pivot pivot;
    private final Funnel funnel;
    private final Leds[] ledStrips = Leds.LED_STRIPS;
    private final Swerve swerve;

    private final GripperCommands gripperCMDs;
    private final PivotCommands pivotCMDs;
    private final FunnelCommands funnelCMDs;
    private final SwerveCommands swerveCMDs;

    private final Color color00bebe = new Color(0, 190, 190);

    public AllCommands(Gripper gripper, Pivot pivot, Funnel funnel, Swerve swerve) {
        this.gripper = gripper;
        this.pivot = pivot;
        this.funnel = funnel;
        this.swerve = swerve;

        this.gripperCMDs = new GripperCommands(gripper);
        this.pivotCMDs = new PivotCommands(pivot);
        this.funnelCMDs = new FunnelCommands(funnel);
        this.swerveCMDs = new SwerveCommands(swerve);
    }

    public Command setManualColor() {
        return LedsCommands.getStaticColorCommand(Color.kAqua, ledStrips);
        // return Commands.either(LedsCommands.getStaticColorCommand(Color.kGreen,
        // ledStrips),
        // Commands.either(
        // LedsCommands.getStaticColorCommand(Color.kYellow, ledStrips),
        // Commands.either(LedsCommands.getStaticColorCommand(Color.kRed, ledStrips),
        // LedsCommands.clearLeds(ledStrips),
        // () -> funnel.getIsCoralIn() && !gripper.getIsCoralIn()),
        // () -> funnel.getIsCoralIn() && gripper.getIsCoralIn()),
        // () -> gripper.getIsCoralIn() && !funnel.getIsCoralIn());
    }

    public Command clearLeds() {
        return LedsCommands.clearLeds(ledStrips);
    }

    public Command scoreLedsCommand() {
        return LedsCommands.colorForSeconds(Color.kBlue, 1, ledStrips)
                .withName("scoreLedsCommand");
    }

    public Command setAlignToReefColor() {
        return LedsCommands.colorForSeconds(Color.kChocolate, LedsConstants.SECONDS_FOR_LEDS_DEFAULT, ledStrips);
    }

    public Command wizardLedsNext() {
        return LedsCommands.colorForSeconds(Color.kWhite, LedsConstants.SECONDS_FOR_LEDS_DEFAULT, ledStrips);
    }

    public Command moveToAngleLedsCommand() {
        return LedsCommands.colorForSeconds(Color.kPurple, LedsConstants.SECONDS_FOR_LEDS_DEFAULT, ledStrips);
    }

    public Command driveLeds() {
        return LedsCommands.colorForSeconds(color00bebe, LedsConstants.SECONDS_FOR_LEDS_DEFAULT, ledStrips);
    }

    public Command intake() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE)
                .alongWith(Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE))
                        .andThen(funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED)
                                .andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
                                        .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE,
                                                GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE)))
                                .until(() -> !funnel.getIsCoralIn() && gripper.getIsCoralIn()))
                        .andThen(LedsCommands.colorForSeconds(Color.kGreen, LedsConstants.SECONDS_FOR_LEDS_DEFAULT,
                                ledStrips)))
                .finallyDo((intterapted) -> {
                    funnel.stop();
                    gripper.stop();
                })
                .withName("Intake");
    }

    public Command autoDrive() {
        return Commands.runOnce(
                () -> swerve.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(swerve.getIsRedAlliance() ? 0 : 180))))
                .andThen(swerveCMDs.driveForwardVoltage(() -> 0.2).withTimeout(1.5)).withName("autoDrive");
    }

    public Command autoDriveScoreL1() {
        return Commands.runOnce(
                () -> swerve.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(swerve.getIsRedAlliance() ? 0 : 180))))
                .andThen(swerveCMDs.driveForwardVoltage(() -> 0.2).withTimeout(1.5))
                .andThen(gripperCMDs.score(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
                .withName("autoDriveScoreL1");
    }

    public Command autoMoveToL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
                .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L1))
                .withName("autoMoveToL3");
    }

    public Command autoScoreL1() {
        return gripperCMDs.score(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
                .until(() -> !gripper.getIsCoralIn()).withName("scoreL1");
    }

    public Command autoMoveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2).withName("autoMoveToL3");
    }

    public Command moveToL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
                .alongWith(
                        Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L1)).andThen(moveToAngleLedsCommand()))
                .withName("moveToL1");
    }

    public Command intakeStatic() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
                .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE,
                        GRIPPER_LEFT_LOADING_VOLTAGE))
                .alongWith(scoreLedsCommand()).withName("moveToL1Static");
    }

    public Command moveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2)
                .alongWith(
                        Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L2)).andThen(moveToAngleLedsCommand()))
                .withName("moveToL2");
    }

    public Command moveToL3() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L3)
                .alongWith(
                        Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L3)).andThen(moveToAngleLedsCommand()))
                .withName("moveToL3");
    }

    public Command moveToRest() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_REST)
                .alongWith(Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L3))
                        .andThen(LedsCommands.colorForSeconds(Color.kDarkOrange, LedsConstants.SECONDS_FOR_LEDS_DEFAULT,
                                ledStrips)))
                .withName("moveToRest");
    }

    public Command scoreL1() {
        return gripperCMDs.score(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
        .alongWith(Commands.waitUntil(() -> !gripper.getIsCoralIn()).andThen(Commands.runOnce(() -> scoreLedsCommand())))
        .finallyDo(() -> {
            gripper.stop();
        }).withName("scoreL1");
    }

    public Command scoreL3() {
        return gripperCMDs.score(GRIPPER_BACK_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE)
            .alongWith(Commands.waitUntil(() -> !gripper.getIsCoralIn()).andThen(() -> scoreLedsCommand()))
            .finallyDo(() -> {
                gripper.stop();
            }).withName("scoreL3");
    }

    public Command alignToReefRight(TuneableCommand driveCommand, BooleanSupplier lockOnPose) {
        return new DeferredCommand(() -> swerveCMDs.driveToPoseWithPID(
                lockOnPose.getAsBoolean() ? swerve.getLastCalculatedClosestPose()
                        : swerve.getClosestPose(FieldConstants.REEF_RIGHT_BRANCHES_POSES),
                driveCommand).andThen(setAlignToReefColor()), Set.of(swerve))
                .onlyWhile(() -> swerve.getDistanceToPose(
                        swerve.getLastCalculatedClosestPose()) <= SwerveContants.AlignToReef.MIN_DISTANCE_TO_AMPALIGN);
    }

    public Command alignToReefLeft(TuneableCommand driveCommand, BooleanSupplier lockOnPose) {
        return new DeferredCommand(() -> swerveCMDs.driveToPoseWithPID(
                lockOnPose.getAsBoolean() ? swerve.getLastCalculatedClosestPose()
                        : swerve.getClosestPose(FieldConstants.REEF_LEFT_BRANCHES_POSES),
                driveCommand).andThen(setAlignToReefColor()), Set.of(swerve))
                .onlyWhile(() -> swerve.getDistanceToPose(
                        swerve.getLastCalculatedClosestPose()) <= SwerveContants.AlignToReef.MIN_DISTANCE_TO_AMPALIGN);
    }

    public TuneableCommand getPivotReadyAndScore() {
        return TuneableCommand.wrap((tuneableTable) -> {
            DoubleHolder angleHolder = tuneableTable.addNumber("angle", PIVOT_TUNEABLE_ANGLE);
            DoubleHolder backGripperVoltage = tuneableTable.addNumber("back gripper voltage",
                    GRIPPER_BACK_TUNEABLE_VOLTAGE);
            DoubleHolder leftGripperVoltage = tuneableTable.addNumber("left gripper voltage",
                    GRIPPER_LEFT_TUNEABLE_VOLTAGE);
            DoubleHolder rightGripperVoltage = tuneableTable.addNumber("right gripper voltage",
                    GRIPPER_RIGHT_TUNEABLE_VOLTAGE);

            return (pivotCMDs.moveToAngle(angleHolder.get()))
                    .andThen(gripperCMDs.score(backGripperVoltage.get(), rightGripperVoltage.get(),
                            leftGripperVoltage.get()))
                    .withName("getPivotAngleAndScore");
        });
    }

    /*
     * The Expected Behavior:
     * 1. Manual funnel conntroller
     * 2. Manual gripper conntroller
     * 3. Manual pivot conntroller
     * 4. Pivot move to intake
     * 5. Funnel load coral
     * 6. Funnel pass coral and gripper load coral
     * 7. Pivot move to L1
     * 8. Score L1
     * 9. Pivot move to intake + intake
     * 11. Pivot move to L3
     * 12. Score L3
     * 13. Pivot move to L2
     * 14. Pivot move to rest (-90)
     * 15. Leds blink *color* (not finished)
     */

    public TuneableCommand testWizard(BooleanSupplier moveToNext, DoubleSupplier firstSpeed, DoubleSupplier secondSpeed,
            DoubleSupplier thirdSpeed) {
        return TuneableCommand.wrap((tuneableBuilder) -> {
            Command command =

                    Commands.print("Manual funnel conntroller")
                            .andThen(manualFunnelController(firstSpeed)
                                    .withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Manual gripper conntroller"))
                            .andThen(gripperCMDs.manualController(
                                    () -> firstSpeed.getAsDouble() * ManualControllers.GRIPPER_BACK_SPEED_MULTIPLAYER,
                                    () -> secondSpeed.getAsDouble() * ManualControllers.GRIPPER_RIGHT_SPEED_MULTIPLAYER,
                                    () -> thirdSpeed.getAsDouble() * ManualControllers.GRIPPER_LEFT_SPEED_MULTIPLAYER)
                                    .withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Manual pivot conntroller"))
                            .andThen(manualPivotController(firstSpeed)
                                    .withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Pivot move to intake"))
                            .andThen(pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE)
                                    .withDeadline(Commands.waitUntil(moveToNext).andThen(wizardLedsNext())

                                            // .andThen(Commands.print("Funnel load coral"))
                                            // .andThen(funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED)
                                            // .withDeadline(Commands.waitUntil(moveToNext)))
                                            // .andThen(wizardLedsNext())

                                            .andThen(Commands.print("Funnel pass coral and gripper load coral"))
                                            .andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
                                                    .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE,
                                                            GRIPPER_RIGHT_LOADING_VOLTAGE,
                                                            GRIPPER_LEFT_LOADING_VOLTAGE))
                                                    .withDeadline(Commands.waitUntil(moveToNext)))
                                            .andThen(wizardLedsNext())))

                            .andThen(Commands.print("Pivot move to L1"))
                            .andThen(moveToL1().withDeadline(
                                    Commands.waitUntil(moveToNext)).andThen(wizardLedsNext())
                                    .andThen(Commands.print("Score L1"))
                                    .andThen(scoreL1()
                                            .withDeadline(Commands.waitUntil(moveToNext)))
                                    .andThen(wizardLedsNext()))

                            .andThen(Commands.print("Pivot move to intake + intake"))
                            .andThen(intake()
                                    .withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Move to L3"))
                            .andThen(moveToL3().withDeadline(
                                    Commands.waitUntil(moveToNext).andThen(wizardLedsNext())
                                            .andThen(Commands.print("Score L3"))
                                            .andThen(scoreL3().withDeadline(Commands.waitUntil(moveToNext)))
                                            .andThen(wizardLedsNext())))

                            .andThen(Commands.print("Move to L2"))
                            .andThen(moveToL2().withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Move to rest"))
                            .andThen(moveToRest().withDeadline(Commands.waitUntil(moveToNext)))
                            .andThen(wizardLedsNext())

                            .andThen(Commands.print("Finished!"))
                            .andThen(LedsCommands.colorForSeconds(color00bebe, 1, ledStrips))

                            .withName("testWizard").withInterruptBehavior(InterruptionBehavior.kCancelIncoming);

            command.addRequirements(pivot, funnel, gripper, swerve);
            return command;
        });
    }

    public Command manualGripperController(DoubleSupplier speed) {
        return gripperCMDs.manualController(
                () -> speed.getAsDouble() * ManualControllers.GRIPPER_BACK_SPEED_MULTIPLAYER,
                () -> speed.getAsDouble() * ManualControllers.GRIPPER_RIGHT_SPEED_MULTIPLAYER,
                () -> speed.getAsDouble() * ManualControllers.GRIPPER_LEFT_SPEED_MULTIPLAYER);
    }

    public Command manualFunnelController(DoubleSupplier speed) {
        return funnelCMDs.manualController(() -> speed.getAsDouble() * ManualControllers.FUNNEL_SPEED_MULTIPLAYER);
    }

    public Command manualPivotController(DoubleSupplier speed) {
        return pivotCMDs.manualController(() -> speed.getAsDouble() * ManualControllers.PIVOT_SPEED_MULTIPLAYER);
    }

    public Command stopAll() {
        return Commands.runOnce(() -> {
            gripper.stop();
            pivot.stop();
            funnel.stop();
            LedsCommands.clearLeds(ledStrips);
        }, gripper, pivot, funnel)
                .ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
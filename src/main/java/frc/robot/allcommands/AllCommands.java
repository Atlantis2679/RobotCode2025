package frc.robot.allcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.allcommands.AllCommandsConstants.ManualControllers;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotCommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

public class AllCommands {
    private final Gripper gripper;
    private final Pivot pivot;
    private final Funnel funnel;

    private final GripperCommands gripperCMDs;
    private final PivotCommands pivotCMDs;
    private final FunnelCommands funnelCMDs;

    public AllCommands(Gripper gripper, Pivot pivot, Funnel funnel) {
        this.gripper = gripper;
        this.pivot = pivot;
        this.funnel = funnel;

        this.gripperCMDs = new GripperCommands(gripper);
        this.pivotCMDs = new PivotCommands(pivot);
        this.funnelCMDs = new FunnelCommands(funnel);
    }
    public Command intake() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE).until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE)).andThen(
            funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE)))
            .until(() -> !funnel.getIsCoralIn() && gripper.getIsCoralIn())).withName("Intake");
    }

    public Command intakeStatic() {
        return funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).withName("intakeStatic");
    }

    public Command moveToL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
            .withName("moveToL1");
    }

    public Command moveToL1Static() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE))
            .withName("moveToL1Static");
    }

    public Command moveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2).withName("moveToL2");
    }

    public Command moveToL3() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L3).withName("moveToL3");
    }

    public Command scoreL1() {
        return gripperCMDs.score(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
            .withName("scoreL1");
    }
    public Command scoreL3(){
        return gripperCMDs.score(GRIPPER_BACK_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE)
            .withName("scoreL3");
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
            .andThen(gripperCMDs.score(backGripperVoltage.get(), rightGripperVoltage.get(), leftGripperVoltage.get()))
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
     * 9. Pivot move to intake
     * 10. Funnel load and pass and Gripper load coral
     * 11. Pivot move to scoreL3
     * 12. Score L3
     * 13. Pivot move to L2
     * 14. Pivot move to rest
     */

    public TuneableCommand testWizard(BooleanSupplier moveToNext, DoubleSupplier firstSpeed, DoubleSupplier secondSpeed, DoubleSupplier thirdSpeed) {
        return TuneableCommand.wrap(tuneableTable -> 
            manualFunnelController(firstSpeed).until(moveToNext)
            .andThen(gripperCMDs.manualController(
                    () -> firstSpeed.getAsDouble() * ManualControllers.GRIPPER_BACK_SPEED_MULTIPLAYER,
                    () -> secondSpeed.getAsDouble() * ManualControllers.GRIPPER_RIGHT_SPEED_MULTIPLAYER, 
                    () -> thirdSpeed.getAsDouble() * ManualControllers.GRIPPER_LEFT_SPEED_MULTIPLAYER))
            .until(moveToNext).andThen(manualPivotController(firstSpeed)).until(moveToNext)
            .andThen(pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE)).andThen(Commands.waitUntil(moveToNext))
            .andThen(funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED)).until(moveToNext)
            .andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE)))
            .until(moveToNext).andThen(moveToL1()).andThen(Commands.waitUntil(moveToNext)).andThen(scoreL1())
            .until(moveToNext).andThen(intake()).andThen(Commands.waitUntil(moveToNext)).andThen(moveToL3())
            .andThen(Commands.waitUntil(moveToNext)).andThen(scoreL3()).until(moveToNext)
            .andThen(moveToL2()).until(moveToNext).andThen(pivotCMDs.moveToAngle(-90)).until(moveToNext)
            .withName("testWizard")
        );
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
        return Commands.run(() -> {
            gripper.stop();
            pivot.stop();
            funnel.stop();
        }, gripper, pivot, funnel)
        .ignoringDisable(true).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
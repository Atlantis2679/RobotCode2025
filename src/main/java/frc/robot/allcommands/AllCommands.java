package frc.robot.allcommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
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
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE)).andThen(
            funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))).until( () -> !funnel.getIsCoralIn() && gripper.getIsCoralIn()))
            .withName("Intake");
    }

    public Command intakeStatic() {
        return funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).withName("intakeStatic");
    }

    public Command moveToL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L1))
        // .andThen(gripperCMDs.score(GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
        // .finallyDo(pivot::stop)
        .withName("moveToL1");
    }

    public Command moveToL1Static() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(gripperCMDs.score(GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
            .withName("scoreStaticL1");
    }

    public Command moveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L2));
    }

    public Command moveToL3() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L3)
        .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_L3));
    }

    public Command scoreL1() {
        return gripperCMDs.score(GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
                .andThen(gripper::stop)
                .finallyDo(pivot::stop);
    }
    public Command scoreL3(){
        return gripperCMDs.score(GRIPPER_L3_VOLTAGE, GRIPPER_L3_VOLTAGE)
            .andThen(gripper::stop)
            .finallyDo(pivot::stop);
    }

    public TuneableCommand getPivotReadyAndScore() {
            return TuneableCommand.wrap((tuneableTable) -> {
            DoubleHolder angleHolder = tuneableTable.addNumber("angle", PIVOT_TUNEABLE_ANGLE);
            DoubleHolder leftGripperVoltage = tuneableTable.addNumber("upper roller speed",
                    GRIPPER_LEFT_TUNEABLE_VOLTAGE);
            DoubleHolder rightGripperVoltage = tuneableTable.addNumber("lower roller speed",
                    GRIPPER_RIGHT_TUNEABLE_VOLTAGE);
            return (pivotCMDs.moveToAngle(angleHolder.get()))
            .andThen(gripperCMDs.score(rightGripperVoltage.get(), leftGripperVoltage.get()))
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
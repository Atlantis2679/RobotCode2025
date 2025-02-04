package frc.robot.allcommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE).andThen(funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED)).withName("Intake");
    }

    public Command intakeStatic() {
        return funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).withName("intakeStatic");
    }

    public Command scoreL1() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1))
            .andThen(gripperCMDs.score(GRIPPER_LOADING_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
            .withName("scoreL1");
    }

    public Command scoreStaticL1() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(gripperCMDs.score(GRIPPER_LOADING_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE))
            .withName("scoreStaticL1");
    }

    public Command scoreL2() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(gripperCMDs.score(GRIPPER_LOADING_VOLTAGE, GRIPPER_L2_VOLTAGE, GRIPPER_L2_VOLTAGE))
            .withName("scoreL2");
    }

    public Command scoreL3() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED).alongWith(gripperCMDs.loadCoral(GRIPPER_INTAKE_VOLTAGE))
            .andThen(gripperCMDs.score(GRIPPER_LOADING_VOLTAGE, GRIPPER_L3_VOLTAGE, GRIPPER_L3_VOLTAGE))
            .withName("scoreL3");
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
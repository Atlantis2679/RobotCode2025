package frc.robot.allcommands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.lib.valueholders.DoubleHolder;
import frc.robot.allcommands.AllCommandsConstants.ManualControllers;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.leds.LedsCommands;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotCommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

public class AllCommands {
    private final Gripper gripper;
    private final Pivot pivot;
    private final Funnel funnel;
    private final Leds[] ledStrips = Leds.LED_STRIPS;

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

    public Command setManualColor(){
        if(gripper.getIsCoralIn() && !funnel.getIsCoralIn()){
            return LedsCommands.colorForSeconds(Color.kGreen,0.1, ledStrips);
        }
        else if(funnel.getIsCoralIn() && !gripper.getIsCoralIn()){
            return LedsCommands.colorForSeconds(Color.kRed,0.1, ledStrips);
        }
        else if(funnel.getIsCoralIn() && gripper.getIsCoralIn()){
            return LedsCommands.colorForSeconds(Color.kYellow,0.1, ledStrips);
        }
        else{
            Color color00bebe = new Color(0, 190, 190);
            return LedsCommands.colorForSeconds(color00bebe,0.1, Leds.LED_STRIPS);
        }
    }

    public Command clearLeds(){
        return LedsCommands.clearLeds(ledStrips);
    }

    public Command scoreLedsCommand(){
        return LedsCommands.colorForSeconds(Color.kBlue, 1, ledStrips)
        .withName("scoreLedsCommand");
    }

    public Command setAlignToReefColor() {
        return LedsCommands.colorForSeconds(Color.kChocolate, 1, ledStrips);
    }

    public Command moveToAngleLedsCommand(){
        return LedsCommands.colorForSeconds(Color.kPurple, 1, ledStrips);
    }

    public Command intake() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE).until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE)).andThen(
            funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).andThen(funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE)))
            .until(() -> !funnel.getIsCoralIn() && gripper.getIsCoralIn()))
            .andThen(LedsCommands.colorForSeconds(Color.kGreen, 1, ledStrips))
            .withName("Intake");
    }

    public Command intakeStatic() {
        return funnelCMDs.loadCoral(FUNNEL_INTAKE_SPEED).withName("intakeStatic");
    }

    public Command moveToL1() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L1)
            .andThen(moveToAngleLedsCommand()).withName("moveToL1");
    }

    public Command moveToL1Static() {
        return funnelCMDs.passCoral(FUNNEL_INTAKE_SPEED, FUNNEL_PASSING_SPEED)
            .alongWith(gripperCMDs.loadCoral(GRIPPER_BACK_LOADING_VOLTAGE, GRIPPER_RIGHT_LOADING_VOLTAGE, GRIPPER_LEFT_LOADING_VOLTAGE))
            .andThen(scoreLedsCommand()).withName("moveToL1Static");
    }

    public Command moveToL2() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L2)
            .andThen(moveToAngleLedsCommand()).withName("moveToL2");
    }

    public Command moveToL3() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_L3)
        .andThen(moveToAngleLedsCommand()).withName("moveToL3");
    }

    public Command scoreL1() {
        return gripperCMDs.score(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
            .andThen(scoreLedsCommand()).withName("scoreL1");
    }
    public Command scoreL3(){
        return gripperCMDs.score(GRIPPER_BACK_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE)
            .andThen(scoreLedsCommand()).withName("scoreL3");
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
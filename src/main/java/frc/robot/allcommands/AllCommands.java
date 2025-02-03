package frc.robot.allcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
    private final GripperCommands gripperCMDs;
    private final PivotCommands pivotCMDs;
    private final FunnelCommands funnelCMDs;

    public AllCommands(Gripper gripper, Pivot pivot, Funnel funnel) {
        this.gripper = gripper;
        this.pivot = pivot;
        this.funnel = funnel;

        gripperCMDs = new GripperCommands(gripper);
        pivotCMDs = new PivotCommands(pivot);
        funnelCMDs = new FunnelCommands(funnel);
    }
    public Command ScoreL3(){
        return Commands.waitUntil(() -> pivot.isAtAngle(L3_Angle))
        .andThen(gripperCMDs.scoreL3()).withName("ScoreL3");
    }

    public Command ScoreL1(){
        return Commands.waitUntil(() -> pivot.isAtAngle(L1_Angle))
        .andThen(gripperCMDs.scoreL1()).withName("ScoreL1");
    }

    public Command ScoreL1Instantly(){
        return gripperCMDs.scoreL1().withName("ScoreL1Instantly");
    }

    public Command ScoreL3Instantly(){
        return gripperCMDs.scoreL3().withName("ScoreL3Instantly");
    }

    public Command delivery() {
        return pivotCMDs.moveToAngle(0).until(() -> pivot.isAtAngle(Delivery_Angle))
                .andThen(gripperCMDs.scoreL3());
    }

    public Command pivotReadyToCollect() {
        return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE).withName("pivotReadyToCollect");
    }

    public Command loadCoral(){
        return funnelCMDs.loadCoral();
    }

    public Command passCoral(){
        return Commands.waitUntil(() -> pivot.isAtAngle(L1_Angle))
        .andThen(funnelCMDs.passCoral()).withName("passCoral");
    }

    public Command stopAll() {
        return Commands.run(() -> {
            gripper.stop();
            pivot.stop();
            funnel.stop();
        }, gripper, pivot, funnel).ignoringDisable(true).withName("stopAll");
    }
}

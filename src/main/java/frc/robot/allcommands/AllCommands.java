package frc.robot.allcommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
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
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;

import static frc.robot.allcommands.AllCommandsConstants.*;

public class AllCommands {
        private final Gripper gripper;
        private final Pivot pivot;
        private final Funnel funnel;
        private final Swerve swerve;
        private final Leds leds;

        private final GripperCommands gripperCMDs;
        private final PivotCommands pivotCMDs;
        private final FunnelCommands funnelCMDs;
        private final SwerveCommands swerveCMDs;
        private final LedsCommands ledsCMDs;

        public AllCommands(Gripper gripper, Pivot pivot, Funnel funnel, Swerve swerve, Leds leds) {
                this.gripper = gripper;
                this.pivot = pivot;
                this.funnel = funnel;
                this.swerve = swerve;
                this.leds = leds;

                this.gripperCMDs = new GripperCommands(gripper);
                this.pivotCMDs = new PivotCommands(pivot);
                this.funnelCMDs = new FunnelCommands(funnel);
                this.swerveCMDs = new SwerveCommands(swerve);
                this.ledsCMDs = new LedsCommands(leds);
        }

        public Command intake() {
                return Commands.parallel(
                                pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_INTAKE),
                                Commands.sequence(
                                                Commands.waitUntil(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_INTAKE)),
                                                funnelCMDs.passCoral(FUNNEL_PRECENTAGE_SPEED)
                                                                .alongWith(gripperCMDs.spin(
                                                                                GRIPPER_BACK_LOADING_VOLTAGE,
                                                                                GRIPPER_RIGHT_LOADING_VOLTAGE,
                                                                                GRIPPER_LEFT_LOADING_VOLTAGE))))
                                .until(() -> !funnel.getIsCoralIn()
                                                && gripper.getIsCoralIn())
                                .andThen(new ScheduleCommand(
                                                ledsCMDs.blink(Color.kBlue, LEDS_BLINK_DEFAULT_SEC)))

                                .withName("Intake");
        }

        public Command autoDrive() {
                return Commands.runOnce(
                                () -> swerve.resetPose(new Pose2d(0, 0,
                                                Rotation2d.fromDegrees(swerve.getIsRedAlliance() ? 0 : 180))))
                                .andThen(swerveCMDs.driveForwardVoltage(() -> AUTO_DRIVE_VOLTAGE_PERCANTAGE)
                                                .withTimeout(AUTO_DRIVE_SECONDS))
                                .withName("NotRealAutoDrive");
        }

        public Command autoDriveScoreL1() {
                return Commands.runOnce(
                                () -> swerve.resetPose(new Pose2d(0, 0,
                                                Rotation2d.fromDegrees(swerve.getIsRedAlliance() ? 0 : 180))))
                                .andThen(swerveCMDs.driveForwardVoltage(() -> AUTO_DRIVE_VOLTAGE_PERCANTAGE)
                                                .withTimeout(AUTO_DRIVE_SECONDS))
                                .andThen(gripperCMDs
                                                .spin(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE,
                                                                GRIPPER_LEFT_L1_VOLTAGE)
                                                .withTimeout(1.5))
                                .withName("NotRealAutoDriveScoreL1");
        }

        public Command pivotToAngleWithLeds(double angle) {
                return Commands.parallel(
                                pivotCMDs.moveToAngle(angle),
                                ledsCMDs.staticColorWhenTrue(() -> pivot.isAtAngle(angle), Color.kGreen))
                                .withName("pivotToAngleWithLeds");
        }

        public Command moveToL1() {
                return pivotToAngleWithLeds(PIVOT_ANGLE_FOR_L1).withName("moveToL1");
        }

        public Command moveToL2() {
                return pivotToAngleWithLeds(PIVOT_ANGLE_FOR_L2).withName("moveToL2");
        }

        public Command moveToL3() {
                return pivotToAngleWithLeds(PIVOT_ANGLE_FOR_L3).withName("moveToL3");
        }

        public Command moveToRest() {
                return pivotCMDs.moveToAngle(PIVOT_ANGLE_FOR_REST).finallyDo(pivot::stop)
                                .until(() -> pivot.isAtAngle(PIVOT_ANGLE_FOR_REST))
                                .andThen(Commands.waitUntil(
                                                () -> Math.abs(pivot.getAngleDegrees() - PIVOT_ANGLE_FOR_REST) > 17))
                                .repeatedly()
                                .withName("moveToRest");
        }

        public Command scoreL1() {
                return gripperCMDs.spin(GRIPPER_BACK_L1_VOLTAGE, GRIPPER_RIGHT_L1_VOLTAGE, GRIPPER_LEFT_L1_VOLTAGE)
                                .withName("scoreL1");
        }

        public Command scoreL3() {
                return gripperCMDs.spin(GRIPPER_BACK_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE, GRIPPER_OUTTAKE_L3_VOLTAGE)
                                .withName("scoreL3");
        }

        public TuneableCommand movePivotToAngleTuneable() {
                return TuneableCommand.wrap((tuneableTable) -> {
                        DoubleHolder angleHolder = tuneableTable.addNumber("angle", 0.0);

                        return pivotCMDs.moveToAngle(angleHolder::get).withName("getPivotAngleAndScore");
                });
        }

        public Command manualGripperController(DoubleSupplier speed) {
                return gripperCMDs.manualController(
                                () -> speed.getAsDouble() * ManualControllers.GRIPPER_BACK_SPEED_MULTIPLAYER,
                                () -> speed.getAsDouble() * ManualControllers.GRIPPER_RIGHT_SPEED_MULTIPLAYER,
                                () -> speed.getAsDouble() * ManualControllers.GRIPPER_LEFT_SPEED_MULTIPLAYER);
        }

        public Command manualFunnelController(DoubleSupplier speed) {
                return funnelCMDs.manualController(
                                () -> speed.getAsDouble() * ManualControllers.FUNNEL_SPEED_MULTIPLAYER);
        }

        public Command manualPivotController(DoubleSupplier speed) {
                return pivotCMDs.manualController(
                                () -> speed.getAsDouble() * ManualControllers.PIVOT_SPEED_MULTIPLAYER);
        }

        public Command manualConntroller(BooleanSupplier scoreL1, BooleanSupplier scoreL3,
                        DoubleSupplier pivotSpeed, DoubleSupplier funnelGripperSpeed) {
                return Commands.parallel(
                                manualFunnelController(funnelGripperSpeed),
                                AllCommands.dynamicSwitchBetweenCommands(
                                                () -> scoreL1.getAsBoolean() || scoreL3.getAsBoolean(),
                                                AllCommands.dynamicSwitchBetweenCommands(
                                                                scoreL1, scoreL3,
                                                                scoreL1(), scoreL3()),
                                                manualGripperController(funnelGripperSpeed)),
                                manualPivotController(pivotSpeed))
                                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
        }

        public Command stopAll() {
                return Commands.run(() -> {
                        gripper.stop();
                        pivot.stop();
                        funnel.stop();
                        leds.clear();
                }, gripper, pivot, funnel);
        }

        public static Command dynamicSwitchBetweenCommands(BooleanSupplier condition, Command onTrue, Command onFalse) {
                return dynamicSwitchBetweenCommands(condition, () -> !condition.getAsBoolean(), onTrue, onFalse);
        }

        public static Command dynamicSwitchBetweenCommands(BooleanSupplier switchToFirst,
                        BooleanSupplier switchToSecond,
                        Command first, Command second) {
                return Commands.repeatingSequence(
                                first.until(switchToSecond),
                                second.until(switchToFirst));
        }
}
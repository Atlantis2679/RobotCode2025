package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.funnel.FunnelCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.gripper.GripperCommands;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Gripper gripper = new Gripper();
    private final Funnel funnel = new Funnel();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
                RobotMap.Controllers.OPERATOR_PORT);    

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final GripperCommands gripperCommands = new GripperCommands(gripper);
    private final FunnelCommands funnelCommands = new FunnelCommands(funnel);

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).onTrue(swerveCommands.stop().repeatedly().withTimeout(0.5));

        configureDriverBindings();
        configureOperatorBindings();
    }

    private void configureDriverBindings() {
        TuneableCommand driveCommand = swerveCommands.driverController(
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> driverController.getRightX(),
                driverController.leftBumper().negate()::getAsBoolean,
                driverController.rightBumper()::getAsBoolean);

        swerve.setDefaultCommand(driveCommand);
        TuneablesManager.add("Swerve/drive command", driveCommand.fullTuneable());
        driverController.a().onTrue(new InstantCommand(swerve::resetYaw));
        driverController.x().onTrue(swerveCommands.xWheelLock());

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        operatorController.a().onTrue(funnelCommands.loadCoral(0.3));
        operatorController.y().onTrue(funnelCommands.loadCoral(0.3).alongWith(gripperCommands.loadCoral(7)));
        operatorController.x().onTrue(gripperCommands.scoreL1(7, 6, 4));
        operatorController.b().onTrue(funnelCommands.passCoral(0.3, 0.35).alongWith(gripperCommands.loadCoral(7))
            .andThen(gripperCommands.scoreL1(7, 6, 4)));
    }
    

    public void setSubsystemsInTestModeState() {
        swerve.enableCoast();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

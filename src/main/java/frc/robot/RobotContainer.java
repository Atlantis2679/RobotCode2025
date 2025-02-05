package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.allcommands.AllCommands;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Pivot pivot = new Pivot();
    private final Gripper gripper = new Gripper();
    private final Funnel funnel = new Funnel();
    private final PowerDistribution pdh = new PowerDistribution();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
                RobotMap.Controllers.OPERATOR_PORT);    
    
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(gripper, pivot, funnel);

    private boolean useStaticCommands = false;

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).onTrue(swerveCommands.stop().repeatedly().withTimeout(0.5));
        pdh.setSwitchableChannel(true);
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
        operatorController.a().onTrue(Commands.either(allCommands.intakeStatic(), allCommands.intake(), () -> useStaticCommands));
        operatorController.leftBumper().onChange(Commands.runOnce(() -> useStaticCommands = !useStaticCommands));
        operatorController.b().onTrue(Commands.either(allCommands.scoreStaticL1(), allCommands.scoreL1(), () -> useStaticCommands));
        operatorController.y().onTrue(allCommands.scoreL2());
        operatorController.x().onTrue(allCommands.scoreL3());
        operatorController.rightBumper().whileTrue(Commands.parallel(
            allCommands.manualFunnelController(operatorController::getLeftY),
            allCommands.manualGripperController(operatorController::getLeftX),
            allCommands.manualPivotController(operatorController::getRightY)
        ));
    }
    

    public void setSubsystemsInTestModeState() {
        swerve.enableCoast();
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}

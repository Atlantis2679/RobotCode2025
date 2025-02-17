package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.Tuneable;
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

    private final SendableChooser<Command> autoChooser;

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
                RobotMap.Controllers.OPERATOR_PORT);    
    
    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(gripper, pivot, funnel, swerve);
    
    private boolean useStaticCommands = false;

    private boolean isCompetition = false;

    private PathPlannerAuto autoCommand;

    public RobotContainer() {
        NamedCommands.registerCommand("intake", allCommands.intake());
        NamedCommands.registerCommand("scoreL1", allCommands.scoreL1());
        NamedCommands.registerCommand("moveToL2", allCommands.moveToL2());
        NamedCommands.registerCommand("moveToL3", allCommands.moveToL3());
        NamedCommands.registerCommand("scoreL3", allCommands.scoreL3());
        NamedCommands.registerCommand("score", allCommands.scoreL1Shoot());

        new Trigger(DriverStation::isDisabled).onTrue(swerveCommands.stop().repeatedly().withTimeout(0.5));
        pdh.setSwitchableChannel(true);

        configureDriverBindings();
        configureOperatorBindings();

        autoChooser =  AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> isCompetition
              ? stream.filter(auto -> auto.getName().startsWith("comp"))
              : stream
          );
          SmartDashboard.putData("Auto Chooser", autoChooser);

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
        driverController.b().whileTrue(allCommands.alignToReef(driveCommand));
        driverController.x().onTrue(swerveCommands.xWheelLock());
        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        operatorController.a().onTrue(allCommands.intake());
        operatorController.leftBumper().onChange(Commands.runOnce(() -> useStaticCommands = !useStaticCommands));
        // operatorController.b().onTrue(Commands.either(allCommands.scoreStaticL1(), allCommands.scoreL1(), () -> useStaticCommands));
        operatorController.b().onTrue(allCommands.scoreL1());
        operatorController.y().onTrue(allCommands.moveToL2());
        operatorController.x().onTrue(allCommands.moveToL3());
        TuneableCommand tuneableAngleAndScore = allCommands.getPivotReadyAndScore();
        // operatorController.x().whileTrue(tuneableAngleAndScore);
        TuneablesManager.add("ready to Angle and score", (Tuneable) tuneableAngleAndScore);
        operatorController.rightTrigger().onTrue(allCommands.scoreL3());
        operatorController.leftTrigger().onTrue(allCommands.scoreL1Shoot());
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
        return autoChooser.getSelected();
    }
}

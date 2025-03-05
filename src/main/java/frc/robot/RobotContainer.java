package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.tuneables.Tuneable;
import frc.lib.tuneables.TuneablesManager;
import frc.lib.tuneables.extensions.TuneableCommand;
import frc.robot.subsystems.funnel.Funnel;
import frc.robot.allcommands.AllCommands;
import frc.robot.subsystems.gripper.Gripper;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveCommands;
import frc.robot.utils.NaturalXboxController;

public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Funnel funnel = new Funnel();
    private final Pivot pivot = new Pivot();
    private final Gripper gripper = new Gripper();
    private final Leds leds = new Leds();

    private final PowerDistribution pdh = new PowerDistribution();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();

    private final NaturalXboxController driverController = new NaturalXboxController(
            RobotMap.Controllers.DRIVER_PORT);
    private final NaturalXboxController operatorController = new NaturalXboxController(
            RobotMap.Controllers.OPERATOR_PORT);

    private final SwerveCommands swerveCommands = new SwerveCommands(swerve);
    private final AllCommands allCommands = new AllCommands(gripper, pivot, funnel, swerve, leds);

    private boolean isCompetition = true;

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop()
                .alongWith(allCommands.stopAll()).ignoringDisable(true));
        pdh.setSwitchableChannel(true);

        CameraServer.startAutomaticCapture().setResolution(300, 300);
        
        configureDriverBindings();
        configureOperatorBindings();
        configureAuto();
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

        TuneableCommand alignToReef = swerveCommands.alignToReef(true);
        driverController.leftTrigger()
                .whileTrue(alignToReef);
        TuneablesManager.add("Swerve/align to reef", alignToReef.fullTuneable());
        driverController.rightTrigger()
                .whileTrue(swerveCommands.alignToReef(false));

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        operatorController.a().whileTrue(allCommands.intake());

        operatorController.b().whileTrue(allCommands.moveToL1());
        operatorController.y().whileTrue(allCommands.moveToL2());
        operatorController.x().whileTrue(allCommands.moveToL3());

        TuneableCommand tuneableMovePivotToAngle = allCommands.movePivotToAngleTuneable();
        operatorController.povUp().and(TuneablesManager::isEnabled).whileTrue(tuneableMovePivotToAngle);

        TuneablesManager.add("pivot move to angle", (Tuneable) tuneableMovePivotToAngle);
        operatorController.rightTrigger().whileTrue(allCommands.scoreL3());
        operatorController.leftTrigger().whileTrue(allCommands.scoreL1());
        operatorController.rightBumper().whileTrue(Commands.parallel(
                allCommands.manualFunnelController(operatorController::getLeftY),
                allCommands.manualGripperController(operatorController::getLeftX),
                allCommands.manualPivotController(operatorController::getRightY)));

        pivot.setDefaultCommand(allCommands.moveToRest());
    }

    public void configureAuto() {
        NamedCommands.registerCommand("intake", allCommands.intake());
        NamedCommands.registerCommand("scoreL3", allCommands.scoreL3());
        NamedCommands.registerCommand("score", allCommands.scoreL1());
        NamedCommands.registerCommand("stopAll", allCommands.stopAll());

        new EventTrigger("intake").whileTrue(allCommands.intake());
        new EventTrigger("moveToL1").whileTrue(allCommands.moveToL1());
        new EventTrigger("moveToL2").whileTrue(allCommands.moveToL2());

        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
                (stream) -> isCompetition
                        ? stream.filter(auto -> auto.getName().startsWith("comp"))
                        : stream);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        Field2d field = new Field2d();

        swerve.registerCallbackOnPoseUpdate((pose, isRedAlliance) -> {
            field.setRobotPose(pose);
        });
        SmartDashboard.putData(field);
        autoChooser.onChange((command) -> {
            if (command.getName() != "None") {
                try {
                    List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(command.getName());
                    List<Pose2d> poses = new ArrayList<>();
                    for (PathPlannerPath path : paths) {
                        List<Pose2d> pathPoses = path.getPathPoses();
                        for (Pose2d pose : pathPoses)
                            poses.add(pose);
                    }
                    field.getObject("Auto Trajectory").setPoses(poses);
                } catch (Exception e) {
                    System.out.println("Auto Trajectory Loading Failed!");
                }
            } else {
                field.getObject("Auto Trajectory").setPose(swerve.getPose());
            }
        });
    }

    public void setSubsystemsInTestModeState() {
        swerve.enableCoast();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
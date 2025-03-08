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
import frc.robot.subsystems.leds.LedsCommands;
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
    private final LedsCommands ledsCommands = new LedsCommands(leds);
    private final AllCommands allCommands = new AllCommands(gripper, pivot, funnel, swerve, leds);

    public RobotContainer() {
        new Trigger(DriverStation::isDisabled).whileTrue(swerveCommands.stop()
                .alongWith(allCommands.stopAll()).ignoringDisable(true));
        pdh.setSwitchableChannel(true);

        new Trigger(DriverStation::isEnabled)
                .whileTrue(Commands.startEnd(gripper::setBreakMotors, gripper::setCoastMotors));

        if (Robot.isReal())
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

        driverController.start()
                .whileTrue(ledsCommands.bebeGradient().asProxy().unless(() -> leds.getCurrentCommand() != null));

        TuneablesManager.add("Swerve/modules control mode",
                swerveCommands.controlModules(
                        driverController::getLeftX,
                        driverController::getLeftY,
                        driverController::getRightY).fullTuneable());
    }

    private void configureOperatorBindings() {
        operatorController.x().whileTrue(allCommands.intake());

        operatorController.a().whileTrue(allCommands.moveToL1());
        operatorController.b().whileTrue(allCommands.moveToL2());
        operatorController.y().whileTrue(allCommands.moveToL3());

        Trigger scoreTrigger = operatorController.rightTrigger().or(operatorController.leftTrigger());
        operatorController.a().and(scoreTrigger).whileTrue(allCommands.scoreL1());
        operatorController.b().and(scoreTrigger).whileTrue(allCommands.scoreL3());
        operatorController.y().and(scoreTrigger).whileTrue(allCommands.scoreL3());

        TuneableCommand tuneableMovePivotToAngle = allCommands.movePivotToAngleTuneable();
        operatorController.povUp().and(TuneablesManager::isEnabled).whileTrue(tuneableMovePivotToAngle);

        TuneablesManager.add("pivot move to angle", (Tuneable) tuneableMovePivotToAngle);

        operatorController.rightBumper().whileTrue(allCommands.manualConntroller(
                operatorController.leftTrigger(),
                operatorController.rightTrigger(),
                operatorController::getRightY,
                operatorController::getLeftY));

        operatorController.start()
            .whileTrue(ledsCommands.rainbow().asProxy().unless(() -> leds.getCurrentCommand() != null));

        pivot.setDefaultCommand(allCommands.moveToRest());

        gripper.setDefaultCommand(allCommands.manualGripperController(operatorController::getLeftY));
        funnel.setDefaultCommand(allCommands.manualFunnelController(operatorController::getLeftY));

        Command pivotDefaultRestLock = Commands
                .runOnce(() -> pivot.setDefaultCommand(pivot.run(pivot::stop).finallyDo(() -> {
                    if (DriverStation.isEnabled())
                        pivot.setDefaultCommand(allCommands.moveToRest());
                })))
                .ignoringDisable(true);

        new Trigger(DriverStation::isDSAttached).onFalse(pivotDefaultRestLock);
        new Trigger(DriverStation::isDisabled).onTrue(pivotDefaultRestLock);
    }

    public void configureAuto() {
        NamedCommands.registerCommand("intake", allCommands.intake());
        NamedCommands.registerCommand("scoreL3", allCommands.scoreL3().withTimeout(1));
        NamedCommands.registerCommand("score", allCommands.scoreL1().withTimeout(1));
        NamedCommands.registerCommand("stopAll", allCommands.stopAll());

        new EventTrigger("intake").onTrue(allCommands.intake());
        new EventTrigger("moveToL1").onTrue(allCommands.moveToL1());
        new EventTrigger("moveToL2").onTrue(allCommands.moveToL2());
        new EventTrigger("rest").onTrue(Commands.runOnce(pivot::stop));

        autoChooser = AutoBuilder.buildAutoChooser();

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

    /* Backup for comp in case the normal PathPlanner auto will be disfunctional.
     * Should be tested more and be used with cation.*/
    public void configureBackupAuto() {
        autoChooser.addOption("Drive Forward No Score", allCommands.autoDrive());
        autoChooser.addOption("Drive Forward Score L1", allCommands.autoDriveScoreL1());
    }

    public void setSubsystemsInTestModeState() {
        swerve.enableCoast();
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
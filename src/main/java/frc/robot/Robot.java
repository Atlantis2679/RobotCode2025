package frc.robot;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.logfields.LogFieldsTable;
import frc.lib.tuneables.TuneablesManager;

public class Robot extends LoggedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    private String getLogPath() {
        if (isReal()) {
            try {
                // prefer a mounted USB drive if one is accessible
                Path usbDirPath = Paths.get("/u").toRealPath();
                if (Files.isWritable(usbDirPath)) {
                    return usbDirPath.toString();
                }
            } catch (IOException ex) {
                // ignored
            }
        }

        File directory = Filesystem.getOperatingDirectory();
        if (isSimulation()) {
            directory = new File(directory, "simlogs");
            directory.mkdir();
        }
        return directory.getAbsolutePath();
    }

    private void initializeAdvantageKit() {
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }

        if (getIsReplay()) {
            System.out.println("******************* Starting replay mode! *******************");
            setUseTiming(false);
            String logPath = LogFileUtil.findReplayLog();
            Logger.setReplaySource(new WPILOGReader(logPath));
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_replay")));
        } else {
            String logPath = getLogPath();

            Logger.addDataReceiver(new WPILOGWriter(logPath));
            Logger.addDataReceiver(new NT4Publisher() {
                @Override
                public void putTable(LogTable table) {
                    if (table.get("RealOutputs/Tuning Mode", false))
                        super.putTable(table);
                }
            });
            LoggedPowerDistribution.getInstance(0, RobotMap.POWER_DISTRIBUATION_TYPE);
        }

        Logger.start();
    }

    // in a method to avoid unreachable code warning.
    private boolean getIsReplay() {
        return isSimulation() && Constants.REPLAY;
    }

    @Override
    public void robotInit() {
        initializeAdvantageKit();
        enableLiveWindowInTest(false);
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        LogFieldsTable.updateAllTables();
        TuneablesManager.update();
        CommandScheduler.getInstance().run();
        Logger.recordOutput("Tuning Mode", TuneablesManager.isEnabled());
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().disable();
        TuneablesManager.enable();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
        CommandScheduler.getInstance().enable();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
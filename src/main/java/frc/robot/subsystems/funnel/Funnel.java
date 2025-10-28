package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import team2679.atlantiskit.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.io.FunneIO;
import frc.robot.subsystems.funnel.io.FunneIOSim;
import frc.robot.subsystems.funnel.io.FunneIOSparkMax;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class Funnel extends SubsystemBase {
    private final FunneIO io;
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final Debouncer isCoralInDebouncer = new Debouncer(FunnelConstants.debounceTimeSec, DebounceType.kBoth);

    public Funnel() {
        fieldsTable.recordOutput("current command", getCurrentCommand() != null ? getCurrentCommand().getName() : "None");

        io = Robot.isReal() ? new FunneIOSparkMax(this.fieldsTable) : new FunneIOSparkMax(this.fieldsTable);//new FunneIOSim(this.fieldsTable);
    }

    public void setMotorPercentageSpeed(double percentageSpeed) {
        fieldsTable.recordOutput("precentage speed", percentageSpeed);
        io.setMotorVolt(MathUtil.clamp(percentageSpeed, -1, 1));
    }
    @Override
    public void periodic(){
        // fieldsTable.update();
        SmartDashboard.putBoolean("CoralInFunnel", getisCoralInPostDebouncer());
        fieldsTable.recordOutput("isCoralIn", getisCoralInPostDebouncer());
    }
    public boolean getisCoralInPostDebouncer() {
        return isCoralInDebouncer.calculate(io.isCoralDetected.getAsBoolean());
    }

    public void stop() {
        fieldsTable.recordOutput("precentage speed", 0.0);
        io.setMotorVolt(0);
    }
}
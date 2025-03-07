package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.io.FunnelIO;
import frc.robot.subsystems.funnel.io.FunnelIOSim;
import frc.robot.subsystems.funnel.io.FunnelIOSparksMax;

import static frc.robot.subsystems.funnel.FunnelConstants.*;

public class Funnel extends SubsystemBase {
    private final FunnelIO io;
    private final LogFieldsTable fieldsTable = new LogFieldsTable(getName());
    private final Debouncer isCoralInDebouncer = new Debouncer(DEBOUNCER_SECONDS, DebounceType.kBoth);

    public Funnel() {
        fieldsTable.recordOutput("current command",
                getCurrentCommand() == null ? "none" : getCurrentCommand().getName());

        io = Robot.isReal() ? new FunnelIOSparksMax(this.fieldsTable) : new FunnelIOSim(this.fieldsTable);
    }

    public void setMotorPercentageSpeed(double percentageSpeed) {
        fieldsTable.recordOutput("precentage speed", percentageSpeed);
        io.setPercentageSpeed(MathUtil.clamp(percentageSpeed, -1, 1));
    }
    @Override
    public void periodic(){
        fieldsTable.update();
        SmartDashboard.putBoolean("CoralInFunnel", getIsCoralIn());
        fieldsTable.recordOutput("CoralInFunnel", getIsCoralIn());
    }
    public boolean getIsCoralIn() {
        return isCoralInDebouncer.calculate(io.isCoralIn.getAsBoolean());
    }

    public void stop() {
        fieldsTable.recordOutput("precentage speed", 0.0);
        io.setPercentageSpeed(0);
    }
}

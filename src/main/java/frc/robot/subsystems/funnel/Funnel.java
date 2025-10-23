package frc.robot.subsystems.funnel;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.io.FunnelIO;
import frc.robot.subsystems.funnel.io.FunnelIOSim;
import frc.robot.subsystems.funnel.io.FunnelIOSparkMax;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Funnel extends SubsystemBase{
    private LogFieldsTable logs = new LogFieldsTable(getName());
    private FunnelIO io = Robot.isReal() ? new FunnelIOSparkMax(logs) : new FunnelIOSim(logs);
    private Debouncer debouncer = new Debouncer(FunnelConstants.DebouncerDelay);

    public void spin(double v){
        io.setVoltage(v);
    }
    public boolean getBeamBreak() {
        return this.debouncer.calculate(this.io.beamBreak.getAsBoolean());
    }

}

package frc.robot.subsystems.funnel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.funnel.funnelIO.FunnelIO;
import frc.robot.subsystems.funnel.funnelIO.FunnelIOSim;
import frc.robot.subsystems.funnel.funnelIO.FunnelIOSparkMax;
import team2679.atlantiskit.logfields.LogFieldsTable;

public class Funnel extends SubsystemBase{
    LogFieldsTable logFieldsTable = new LogFieldsTable(getName());
    FunnelIO io = Robot.isReal() ? new FunnelIOSparkMax(logFieldsTable) : new FunnelIOSim(logFieldsTable);
    Debouncer beamBreakDebouncer = new Debouncer(FunnelConstants.DEBOUNCER_DELAY, DebounceType.kBoth);

    public Funnel(){}

    //Actual methods:
    public boolean isCoralIn(){
        return beamBreakDebouncer.calculate(io.isBeamBreak.getAsBoolean());
    }
    public void setMotorSpeed(double speed){
        logFieldsTable.recordOutput("Precentage speed", speed);
        io.setPrecentageSpeed(MathUtil.clamp(speed, -1, 1));
    }

    //Periodic Command & stop

    @Override
    public void periodic() {
        logFieldsTable.recordOutput("Current Command", getCurrentCommand()!=null?getCurrentCommand().getName():"None");
        SmartDashboard.putBoolean("Funnel:IsCoralIn?", isCoralIn());
        logFieldsTable.recordOutput("isCoralIn", isCoralIn());
    }

    public void stop(){
        setMotorSpeed(0);
    }

}

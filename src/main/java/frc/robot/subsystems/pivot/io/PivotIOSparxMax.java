package frc.robot.subsystems.pivot.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;
import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOSparxMax extends PivotIO {
        private final SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_MOTOR_ID, MotorType.kBrushless);
        private final DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.PIVOT_ENCODER_ID);
        private final SparkMaxConfig config = new SparkMaxConfig();
        public PivotIOSparxMax(LogFieldsTable fieldsTable) {
            super(fieldsTable);
            config.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
            pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        //outputs
        @Override
        protected double getMotorCurrent() {
            return pivotMotor.getOutputCurrent();
        }

        @Override
        protected double getMotorVoltage() {
            return pivotMotor.getAppliedOutput();
        }

        @Override
        protected double getPivotAngleDegrees() {
            return encoder.get();
        }

        //inputs
        @Override
        public void setVoltage(double voltage) {
            pivotMotor.set(voltage);
        }

}
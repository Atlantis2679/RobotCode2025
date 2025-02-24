package frc.robot.subsystems.pivot.io;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.Faults;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;

import static frc.robot.RobotMap.*;

import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOSparkMax extends PivotIO {
        private final SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_MOTOR_ID, MotorType.kBrushless);
        private final DutyCycleEncoder encoder = new DutyCycleEncoder(PIVOT_ENCODER_ID);
        private final SparkMaxConfig config = new SparkMaxConfig();

        private final REVLibError motorConfigError;

        public PivotIOSparkMax(LogFieldsTable fieldsTable) {
            super(fieldsTable);
            config.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
            config.inverted(true);
            motorConfigError = pivotMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        }

        // Outputs:
        @Override
        protected double getMotorCurrent() {
            return pivotMotor.getOutputCurrent();
        }

        @Override
        protected double getMotorVoltage() {
            return pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage();
        }

        @Override
        protected double getPivotAngleDegrees() {
            return encoder.get()*360;
        }

        // Inputs:
        @Override
        public void setVoltage(double voltage) {
            pivotMotor.set(voltage);
        }

        @Override
        protected REVLibError getMotorConfigError() {
            return motorConfigError;
        }

        @Override
        protected Faults getMotorFaults() {
            return pivotMotor.getFaults();
        }

        @Override
        protected Warnings getMotorWarnings() {
            return pivotMotor.getWarnings();
        }
}
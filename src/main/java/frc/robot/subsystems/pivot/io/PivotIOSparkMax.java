package frc.robot.subsystems.pivot.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.subsystems.NetworkAlertsManager;

import static frc.robot.RobotMap.*;

import static frc.robot.subsystems.pivot.PivotConstants.*;

public class PivotIOSparkMax extends PivotIO {
        private final SparkMax pivotLeftMotor = new SparkMax(CANBUS.PIVOT_LEFT_MOTOR_ID, MotorType.kBrushless);
        private final SparkMax pivotRightMotor = new SparkMax(CANBUS.PIVOT_RIGHT_MOTOR_ID, MotorType.kBrushless);
        private final DutyCycleEncoder encoder = new DutyCycleEncoder(PIVOT_ENCODER_ID);
        private final SparkMaxConfig config = new SparkMaxConfig();
        public PivotIOSparkMax(LogFieldsTable fieldsTable) {
            super(fieldsTable);
            config.smartCurrentLimit(PIVOT_CURRENT_LIMIT);
            NetworkAlertsManager.addRevLibErrorAlert("Pivot: Left Motor Config Status: ",
                () -> pivotLeftMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
            NetworkAlertsManager.addRevLibErrorAlert("Pivot: Right Motor Config Status: ",
                () -> pivotRightMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
            NetworkAlertsManager.addSparkMotorAlert("Pivot: Left Motor: ", pivotLeftMotor::getFaults, pivotLeftMotor::getWarnings);
            NetworkAlertsManager.addSparkMotorAlert("Pivot: Right Motor: ", pivotRightMotor::getFaults, pivotLeftMotor::getWarnings);
        }

        // Outputs:
        @Override
        protected double getLeftMotorCurrent() {
            return pivotLeftMotor.getOutputCurrent();
        }

        @Override
        protected double getRightMotorCurrent() {
            return pivotRightMotor.getOutputCurrent();
        }

        @Override
        protected double getLeftMotorVoltage() {
            return pivotLeftMotor.getAppliedOutput() * pivotLeftMotor.getBusVoltage();
        }

        @Override
        protected double getRightMotorVoltage() {
            return pivotRightMotor.getAppliedOutput() * pivotRightMotor.getBusVoltage();
        }

        @Override
        protected double getPivotAngleDegrees() {
            return encoder.get()*360;
        }

        // Inputs:
        @Override
        public void setVoltage(double voltage) {
            pivotLeftMotor.set(voltage);
            pivotRightMotor.set(voltage);
        }

}
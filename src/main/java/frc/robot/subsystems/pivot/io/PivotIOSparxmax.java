package frc.robot.subsystems.pivot.io;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.lib.logfields.LogFieldsTable;
import frc.robot.RobotMap.CANBUS;

public class PivotIOSparxmax extends PivotIO{
        private final SparkMax pivotMotor = new SparkMax(CANBUS.PIVOT_MOTOR_ID, MotorType.kBrushless);
        private final DutyCycleEncoder encoder = new DutyCycleEncoder(CANBUS.PIVOT_ENCODER_ID);

        public PivotIOSparxmax(LogFieldsTable fieldsTable) {
            super(fieldsTable);
        }

        //inputs
        @Override
        protected double getSpeed() {
            return pivotMotor.get();
        }

        @Override
        protected double getPivotAngleDegrees() {
            return encoder.get();
        }
        //outputs
        @Override
        public void setSpeed(double speed) {
            pivotMotor.set(speed);
        }

}
package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FlywheelsIO {
    public static class FlywheelsIOInputs implements LoggableInputs {
        public boolean hasCurrentControl = false;
        public boolean leftMotorConnected = true;
        public boolean rightMotorConnected = true;

        public double leftPositionRotations = 0.0;
        public double leftVelocityRpm = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftSupplyCurrentAmps = 0.0;
        public double leftTorqueCurrentAmps = 0.0;

        public double rightPositionRotations = 0.0;
        public double rightVelocityRpm = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightSupplyCurrentAmps = 0.0;
        public double rightTorqueCurrentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("leftPositionRotations", leftPositionRotations);
            table.put("leftVelocityRpm", leftVelocityRpm);
            table.put("leftAppliedVolts", leftAppliedVolts);
            table.put("leftSupplyCurrentAmps", leftSupplyCurrentAmps);
            table.put("leftTorqueCurrentAmps", leftTorqueCurrentAmps);

            table.put("rightPositionRotations", rightPositionRotations);
            table.put("rightVelocityRpm", rightVelocityRpm);
            table.put("rightAppliedVolts", rightAppliedVolts);
            table.put("rightSupplyCurrentAmps", rightSupplyCurrentAmps);
            table.put("rightTorqueCurrentAmps", rightTorqueCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
            // TODO Auto-generated method stub

        }
    }

    /** Update inputs */
    default void updateInputs(FlywheelsIOInputs inputs) {
    }

    /** Run both motors at voltage */
    default void runVoltage(double voltage) {
    }

    /** Run flywheels at velocity in rpm */
    default void runVelocity(double rpm, double feedforward) {
    }

    /** Stop both flywheels */
    default void stop() {
    }

    /** Config PID values for both motors */
    default void setPID(double kP, double kI, double kD) {
    }

    default void setBrakeMode(boolean enable) {
    }
}
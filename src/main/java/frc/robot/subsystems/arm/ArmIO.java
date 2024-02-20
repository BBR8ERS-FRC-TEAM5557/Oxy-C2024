package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO {

    public static class ArmIOInputs implements LoggableInputs {
        public double armAbsolutePositionDeg = 0.0;
        public double armAbsoluteVelocityDegPerSec = 0.0;

        public double armInternalPositionDeg = 0.0;
        public double armInternalVelocityDegPerSec = 0.0;
        public double[] armAppliedVolts = new double[] {};
        public double[] armSupplyCurrentAmps = new double[] {};
        public double[] armTorqueCurrentAmps = new double[] {};
        public double[] armTempCelsius = new double[] {};

        @Override
        public void toLog(LogTable table) {
            table.put("armAbsolutePositionDeg", armAbsolutePositionDeg);
            table.put("armAbsoluteVelocityDegPerSec", armAbsoluteVelocityDegPerSec);
            table.put("armInternalPositionDeg", armInternalPositionDeg);
            table.put("armInternalVelocityDegPerSec", armInternalVelocityDegPerSec);

            table.put("armAppliedVolts", armAppliedVolts);
            table.put("armSupplyCurrentAmps", armSupplyCurrentAmps);
            table.put("armTorqueCurrentAmps", armTorqueCurrentAmps);
            table.put("armTempCelsius", armTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
        }
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(ArmIOInputs inputs) {
    }

    /** Run the arm open loop at the specified voltage. */
    public default void setVoltage(double volts) {
    }

    public default void setPercent(double percent) {
    }

    public default void setPosition(double angle, double feedforward) {
    }

    public default void stop() {
    }

    public default void resetSensorPosition(double degrees) {
    }

    public default void setBrakeMode(boolean enable) {
    }

    public default void setSoftLimits(boolean enable) {
    }

    default void setPID(double p, double i, double d) {
    }
}

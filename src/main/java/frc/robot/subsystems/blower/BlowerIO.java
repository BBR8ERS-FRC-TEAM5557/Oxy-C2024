package frc.robot.subsystems.blower;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface BlowerIO {
    public static class BlowerIOInputs implements LoggableInputs {
        public double blowerAppliedVolts = 0.0;
        public double blowerCurrentAmps = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("blowerAppliedVolts", blowerAppliedVolts);
            table.put("blowerCurrentAmps", blowerCurrentAmps);
        }

        @Override
        public void fromLog(LogTable table) {
        }

    }

    public default void updateInputs(BlowerIOInputs inputs) {
    }

    public default void setBlowerVoltage(double voltage) {
    }

    public default void setBrakeMode(boolean blowerBrake) {
    }
}

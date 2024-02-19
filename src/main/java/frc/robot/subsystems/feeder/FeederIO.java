package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface FeederIO {

    public static class FeederIOInputs implements LoggableInputs {
        public boolean hasGamepiece = false;

        public double feederPositionRotations = 0.0;
        public double feederVelocityRPM = 0.0;
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
        public double feederTempCelcius = 0.0;

        @Override
        public void toLog(LogTable table) {
            table.put("hasGamepiece", hasGamepiece);
            table.put("feederPositionRotations", feederPositionRotations);
            table.put("feederVelocityRPM", feederVelocityRPM);
            table.put("feederAppliedVolts", feederAppliedVolts);
            table.put("feederCurrentAmps", feederCurrentAmps);
            table.put("feederTempCelcius", feederTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
        }
    }

    public default void updateInputs(FeederIOInputs inputs) {}

    public default void setFeederVoltage(double volts) {}

}
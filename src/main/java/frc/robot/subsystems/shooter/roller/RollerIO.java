package frc.robot.subsystems.shooter.roller;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface RollerIO {

    public static class RollerIOInputs implements LoggableInputs{
        public double RollerVelocityRPM = 0.0;
        public double RollerAppliedVolts = 0.0;
        public double[] RollerCurrentAmps = new double[] {0.0};
        public double[] RollerTempCelcius = new double[] {0.0};

        @Override
        public void toLog(LogTable table) {
            table.put("RollerVelocityRPM", RollerVelocityRPM);
            table.put("RollerAppliedVolts", RollerAppliedVolts);
            table.put("RollerCurrentAmps", RollerCurrentAmps);
            table.put("RollerTempCelcius", RollerTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
            RollerAppliedVolts = table.get("RollerAppliedVolts", RollerAppliedVolts);
            RollerCurrentAmps = table.get("RollerCurrentAmps", RollerCurrentAmps);
            RollerTempCelcius = table.get("RollerTempCelcius", RollerTempCelcius);
        }
    }
        public default void updateInputs(RollerIOInputs inputs) {}

        public default void setRollerVoltage(double voltage) {}
}

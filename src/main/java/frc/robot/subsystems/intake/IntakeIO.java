package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {

    public static class IntakeIOInputs implements LoggableInputs{
        public double IntakeVelocityRPM = 0.0;
        public double IntakeAppliedVolts = 0.0;
        public double[] IntakeCurrentAmps = new double[] {0.0};
        public double[] IntakeTempCelcius = new double[] {0.0};

        @Override
        public void toLog(LogTable table) {
            table.put("IntakeVelocityRPM", IntakeVelocityRPM);
            table.put("IntakeAppliedVolts", IntakeAppliedVolts);
            table.put("IntakeCurrentAmps", IntakeCurrentAmps);
            table.put("IntakeTempCelcius", IntakeTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
            IntakeAppliedVolts = table.get("IntakeAppliedVolts", IntakeAppliedVolts);
            IntakeCurrentAmps = table.get("IntakeCurrentAmps", IntakeCurrentAmps);
            IntakeTempCelcius = table.get("IntakeTempCelcius", IntakeTempCelcius);
        }
    }
        public default void updateInputs(IntakeIOInputs inputs) {}

        public default void setIntakeVoltage(double voltage) {}
}
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IntakeIO {

    public static class IntakeIOInputs implements LoggableInputs {
        public double intakeTopPositionRotations = 0.0;
        public double intakeTopVelocityRPM = 0.0;
        public double intakeTopAppliedVolts = 0.0;
        public double intakeTopCurrentAmps = 0.0;
        public double intakeTopTempCelcius = 0.0;

        public double intakeBottomPositionRotations = 0.0;
        public double intakeBottomVelocityRPM = 0.0;
        public double intakeBottomAppliedVolts = 0.0;
        public double intakeBottomCurrentAmps = 0.0;
        public double intakeBottomTempCelcius = 0.0;


        @Override
        public void toLog(LogTable table) {
            table.put("IntakeTopPositionRotations", intakeTopPositionRotations);
            table.put("IntakeTopVelocityRPM", intakeTopVelocityRPM);
            table.put("IntakeTopAppliedVolts", intakeTopAppliedVolts);
            table.put("IntakeTopCurrentAmps", intakeTopCurrentAmps);
            table.put("IntakeTopTempCelcius", intakeTopTempCelcius);

            table.put("IntakeBottomPositionRotations", intakeBottomPositionRotations);
            table.put("IntakeBottomVelocityRPM", intakeBottomVelocityRPM);
            table.put("IntakeBottomAppliedVolts", intakeBottomAppliedVolts);
            table.put("IntakeBottomCurrentAmps", intakeBottomCurrentAmps);
            table.put("IntakeBottomTempCelcius", intakeBottomTempCelcius);
        }

        @Override
        public void fromLog(LogTable table) {
        }
    }

    public default void updateInputs(IntakeIOInputs inputs) {}

    public default void setIntakeVoltage(double volts) {}

}
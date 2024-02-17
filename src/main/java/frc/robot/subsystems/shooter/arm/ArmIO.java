package frc.robot.subsystems.shooter.arm;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ArmIO {
    public static class ArmIOInputs implements LoggableInputs {
        public double ArmInternalPositionDeg = 0.0;
        public double ArmInternalVelocityDegPerSec = 0.0;
        public double ArmAppliedVolts = 0.0;
        public double[] ArmCurrentAmps = new double[] {0.0};
        public double[] ArmTempCelsius = new double[] {0.0};

        @Override
        public void toLog(LogTable table) {
            table.put("ArmInternalPositionDeg", ArmInternalPositionDeg);
            table.put("ArmInternalVelocityDegPerSec", ArmInternalVelocityDegPerSec);
            table.put("ArmAppliedVolts", ArmAppliedVolts);
            table.put("ArmCurrentAmps", ArmCurrentAmps);
            table.put("ArmTempCelsius", ArmTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            ArmAppliedVolts = table.get("WArmAppliedVolts", ArmAppliedVolts);
            ArmCurrentAmps = table.get("ArmCurrentAmps", ArmCurrentAmps);
            ArmTempCelsius = table.get("ArmTempCelsius", ArmTempCelsius);
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
    
        public default void setAngleDegrees(double targetAngleDegrees, double targetVelocityDegreesPerSec) {
        }
    
        public default void resetSensorPosition(double degreesInches) {
        }
    
        public default void brakeOff() {
        }
    
        public default void brakeOn() {
        }
    
        public default void shouldEnableUpperLimit(boolean value) {
        }
    
        public default void shouldEnableLowerLimit(boolean value) {
        }
}

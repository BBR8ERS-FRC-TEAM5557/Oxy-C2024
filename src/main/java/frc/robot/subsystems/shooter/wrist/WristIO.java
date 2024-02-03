package frc.robot.subsystems.shooter.wrist;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface WristIO {
    public static class WristIOInputs implements LoggableInputs {
        public double WristInternalPositionDeg = 0.0;
        public double WristInternalVelocityDegPerSec = 0.0;
        public double WristAppliedVolts = 0.0;
        public double[] WristCurrentAmps = new double[] {0.0};
        public double[] WristTempCelsius = new double[] {0.0};

        @Override
        public void toLog(LogTable table) {
            table.put("WristInternalPositionDeg", WristInternalPositionDeg);
            table.put("WristInternalVelocityDegPerSec", WristInternalVelocityDegPerSec);
            table.put("WristAppliedVolts", WristAppliedVolts);
            table.put("WristCurrentAmps", WristCurrentAmps);
            table.put("WristTempCelsius", WristTempCelsius);
        }

        @Override
        public void fromLog(LogTable table) {
            WristAppliedVolts = table.get("WristAppliedVolts", WristAppliedVolts);
            WristCurrentAmps = table.get("WristCurrentAmps", WristCurrentAmps);
            WristTempCelsius = table.get("WristTempCelsius", WristTempCelsius);
        }
    }

        /** Updates the set of loggable inputs. */
        public default void updateInputs(WristIOInputs inputs) {
        }
    
        /** Run the Wrist open loop at the specified voltage. */
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

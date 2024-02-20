package frc.robot.subsystems.swerve.module;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface ModuleIO {

  public static class ModuleIOInputs implements LoggableInputs {
    public boolean driveMotorConnected = true;
    public boolean angleMotorConnected = true;
    public boolean hasCurrentControl = false;

    double driveDistanceMeters = 0.0;
    double driveVelocityMetersPerSec = 0.0;
    double driveAppliedVolts = 0.0;
    double driveSupplyCurrentAmps = 0.0;
    double driveTorqueCurrentAmps = 0.0;
    double driveTempCelsius = 0.0;

    double angleAbsolutePositionRad = 0.0;
    double angleInternalPositionRad = 0.0;
    double angleInternalVelocityRadPerSec = 0.0;
    double angleAppliedVolts = 0.0;
    double angleSupplyCurrentAmps = 0.0;
    double angleTorqueCurrentAmps = 0.0;
    double angleTempCelsius = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("DriveDistanceMeters", driveDistanceMeters);
      table.put("DriveVelocityMetersPerSec", driveVelocityMetersPerSec);
      table.put("DriveAppliedVolts", driveAppliedVolts);
      table.put("DriveCurrentAmps", driveSupplyCurrentAmps);
      table.put("DriveTempCelsius", driveTempCelsius);

      table.put("AngleAbsolutePositionRad", angleAbsolutePositionRad);
      table.put("AngleInternalPositionRad", angleInternalPositionRad);
      table.put("AngleInternalVelocityRadPerSec", angleInternalVelocityRadPerSec);
      table.put("AngleAppliedVolts", angleAppliedVolts);
      table.put("AngleCurrentAmps", angleSupplyCurrentAmps);
      table.put("AngleTempCelsius", angleTempCelsius);
    }

    @Override
    public void fromLog(LogTable table) {
    }
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputs inputs) {
  }

  /** Run the drive motor at the specified voltage. */
  public default void setDriveVoltage(double voltage) {
  }

  /** Run the drive motor at the specified velocity. */
  public default void setDriveVelocity(double velocity, double feedforward) {
  }

  /** Run the drive motor at the specified voltage. */
  public default void setAngleVoltage(double voltage) {
  }

  /** Run the turn motor to the specified angle. */
  public default void setAnglePosition(double radians) {
  }

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {
  }

  /** Enable or disable brake mode on the turn motor. */
  public default void setAngleBrakeMode(boolean enable) {
  }

  public default void setAnglePID(double kP, double kI, double kD) {
  }

  public default void setDrivePID(double kP, double kI, double kD) {
  }

  public default boolean resetToAbsolute() {
    return false;
  }

  /** Disable output to all motors */
  default void stop() {
  }
}

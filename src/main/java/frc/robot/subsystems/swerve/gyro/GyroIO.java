package frc.robot.subsystems.swerve.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {

  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("yawPositionRad", yawPosition);
      table.put("yawVelocityRadPerSec", yawVelocityRadPerSec);

    }

    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub

    }
  }

  default void updateInputs(GyroIOInputs inputs) {
  }
}

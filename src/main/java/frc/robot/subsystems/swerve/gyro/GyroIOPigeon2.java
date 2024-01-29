package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix6.hardware.Pigeon2;
//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix6.StatusSignal;
import edu.wpi.first.math.util.Units;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final double[] yprDegrees = new double[3];
  private final double[] xyzDps = new double[3];

  public GyroIOPigeon2() {
    System.out.println("[Init] Creating GyroIOPigeon2");

    pigeon = new Pigeon2(15);

    //pigeon.configFactoryDefault();
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.zeroGyroBiasNow();
    pigeon.setYaw(0.0);
    pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 20);
    
    /** new >> */ pigeon.getPosition().setUpdateFrequency(50);
    
  }

  public void updateInputs(GyroIOInputs inputs) {
    //pigeon.getYawPitchRoll(yprDegrees);
    pigeon.getRawGyro(xyzDps);

    inputs.connected = pigeon.getLastError().equals(ErrorCode.OK);
    //inputs.rollPositionRad = Units.degreesToRadians(yprDegrees[1]);
    inputs.rollPositionRad = Units.degreesToRadians(pigeon.getRoll().getValue());
    //inputs.pitchPositionRad = Units.degreesToRadians(-yprDegrees[2]);
    inputs.pitchPositionRad = Units.degreesToRadians(-pigeon.getPitch().getValue()); //(-) value?
    //inputs.yawPositionRad = Units.degreesToRadians(yprDegrees[0]);
    inputs.yawPositionRad = Units.degreesToRadians(pigeon.getYaw().getValue());

    //>>>>where is xyzDps given a value and what is it actually measuring?
    inputs.rollVelocityRadPerSec = Units.degreesToRadians(xyzDps[1]);
    //inputs.rollVelocityRadPerSec = Units.degreesToRadians(pigeon.get);
    inputs.pitchVelocityRadPerSec = Units.degreesToRadians(-xyzDps[0]);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(xyzDps[2]);
  }
}


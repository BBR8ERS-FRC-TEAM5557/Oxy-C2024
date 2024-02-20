package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class FeederConstants {
    
    public static double kGearReduction = 1.0;

    public static Configuration kMotorConfiguration = new Configuration();
    static {
        kMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kFeederMotor);
        kMotorConfiguration.label = "Feeder Motor";

        kMotorConfiguration.kSmartCurrentLimit = 30.0;
        kMotorConfiguration.kOpenLoopRampRate = 0.25;
        kMotorConfiguration.kShouldInvert = true;
        kMotorConfiguration.kVoltageCompensation = 12.0;
        kMotorConfiguration.kIdleMode = IdleMode.kCoast;
    }
}

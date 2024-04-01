package frc.robot.subsystems.blower;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class BlowerConstants {

    public static Configuration kBlowerMotorConfiguration = new Configuration();
    static {
        kBlowerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kBlowerMotor);
        kBlowerMotorConfiguration.label = "Blower Motor";

        kBlowerMotorConfiguration.kSmartCurrentLimit = 30.0;
        kBlowerMotorConfiguration.kOpenLoopRampRate = 0.5;
        kBlowerMotorConfiguration.kShouldInvert = false;
        kBlowerMotorConfiguration.kVoltageCompensation = 12.0;
        kBlowerMotorConfiguration.kIdleMode = IdleMode.kCoast;
    }
}

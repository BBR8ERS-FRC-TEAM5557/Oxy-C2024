package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class IntakeConstants {

    public static Configuration kMasterIntakeMotorConfiguration = new Configuration();
        static {
            kMasterIntakeMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kLIntakeMotor);
            kMasterIntakeMotorConfiguration.label = "Intake Motor";

            kMasterIntakeMotorConfiguration.kSmartCurrentLimit = 30.0;
            kMasterIntakeMotorConfiguration.kShouldInvert = false;
            kMasterIntakeMotorConfiguration.kOpenLoopRampRate = 0.25;
            kMasterIntakeMotorConfiguration.kIdleMode = IdleMode.kBrake;
            kMasterIntakeMotorConfiguration.kVoltageCompensation = 12.0;

        }

}

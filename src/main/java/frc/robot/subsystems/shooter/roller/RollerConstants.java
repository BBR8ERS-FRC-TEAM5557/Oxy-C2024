package frc.robot.subsystems.shooter.roller;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class RollerConstants {

    public static SparkMaxConfiguration kRollerMotorConfiguration = new SparkMaxConfiguration();
        static {
            kRollerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kRollerMotor);
            kRollerMotorConfiguration.label = "Roller Motor";

            kRollerMotorConfiguration.kSmartCurrentLimit = 30.0;
            kRollerMotorConfiguration.kShouldInvert = false;
            kRollerMotorConfiguration.kOpenLoopRampRate = 0.25;
            kRollerMotorConfiguration.kIdleMode = IdleMode.kBrake;
            kRollerMotorConfiguration.kVoltageCompensation = 12.0;

        }

}

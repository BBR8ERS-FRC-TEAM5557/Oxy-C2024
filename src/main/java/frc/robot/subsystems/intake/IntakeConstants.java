package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.SparkMaxConfiguration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class IntakeConstants {

    public static SparkMaxConfiguration kIntakeMotorConfiguration = new SparkMaxConfiguration();
        static {
            kIntakeMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kIntakeMotor);
            kIntakeMotorConfiguration.label = "Intake Motor";

            kIntakeMotorConfiguration.kSmartCurrentLimit = 30.0;
            kIntakeMotorConfiguration.kShouldInvert = false;
            kIntakeMotorConfiguration.kOpenLoopRampRate = 0.25;
            kIntakeMotorConfiguration.kIdleMode = IdleMode.kBrake;
            kIntakeMotorConfiguration.kVoltageCompensation = 12.0;

        }

}

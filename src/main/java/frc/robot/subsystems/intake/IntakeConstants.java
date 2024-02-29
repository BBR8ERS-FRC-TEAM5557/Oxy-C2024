package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class IntakeConstants {
    public static double kTopGearReduction = 24.0 / 16.0;
    public static double kBottomGearReduction = 24.0 / 16.0;

    public static Configuration kTopMotorConfiguration = new Configuration();
    static {
        kTopMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kIntakeTopMotor);
        kTopMotorConfiguration.label = "Intake Top Motor";

        kTopMotorConfiguration.kSmartCurrentLimit = 60.0;
        kTopMotorConfiguration.kOpenLoopRampRate = 0.25;
        kTopMotorConfiguration.kShouldInvert = true;
        kTopMotorConfiguration.kVoltageCompensation = 12.0;
        kTopMotorConfiguration.kIdleMode = IdleMode.kCoast;
    }

    public static Configuration kBottomMotorConfiguration = new Configuration();
    static {
        kBottomMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, RobotMap.kIntakeBottomMotor);
        kBottomMotorConfiguration.label = "Intake Bottom Motor";

        kBottomMotorConfiguration.kSmartCurrentLimit = 60.0;
        kBottomMotorConfiguration.kOpenLoopRampRate = 0.25;
        kBottomMotorConfiguration.kShouldInvert = true;
        kBottomMotorConfiguration.kVoltageCompensation = 12.0;
        kBottomMotorConfiguration.kIdleMode = IdleMode.kCoast;
    }
}

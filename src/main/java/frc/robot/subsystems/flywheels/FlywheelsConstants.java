package frc.robot.subsystems.flywheels;

import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team5557.factory.TalonFactory;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants.RobotMap;

public class FlywheelsConstants {
    /* Physical Characteristics */
    public static final double kKrakenFreeSpeed = 6000.0;
    public static final double kGearReduction = 24.0 / 40.0;
    public static final double kMaxRPM = kKrakenFreeSpeed / kGearReduction;

    /* PID & FF Values */
    public static final double kFlywheelP = 0.0;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;

    public static final double kFlywheelS = 0.0;
    public static final double kFlywheelV = 12.0 / kMaxRPM; //theoretical
    public static final double kFlywheelA = 0.0;

    public static final double kAllowableError = 200.0;

    /* Left Motor Configuration */
    public static TalonFactory.PIDConfiguration kPIDConfiguration = new TalonFactory.PIDConfiguration();
    static {
        kPIDConfiguration.kP = kFlywheelP;
        kPIDConfiguration.kI = kFlywheelI;
        kPIDConfiguration.kD = kFlywheelD;
    }

    public static TalonFactory.Configuration kLeftMotorConfiguration = new TalonFactory.Configuration();
    static {
        kLeftMotorConfiguration.canID = new CANDeviceId(CANDeviceType.TALON, RobotMap.kFlywheelsLeftMotor);
        kLeftMotorConfiguration.label = "Left Motor";

        kLeftMotorConfiguration.setInverted = true;
        kLeftMotorConfiguration.pid = kPIDConfiguration;
        kLeftMotorConfiguration.neutralMode = NeutralModeValue.Coast;
        kLeftMotorConfiguration.supplyCurrentLimit = 40.0;
    }

    public static TalonFactory.Configuration kRightMotorConfiguration = new TalonFactory.Configuration();
    static {
        kRightMotorConfiguration.canID = new CANDeviceId(CANDeviceType.TALON, RobotMap.kFlywheelRightMotor);
        kRightMotorConfiguration.label = "Right Motor";

        kRightMotorConfiguration.setInverted = false;
        kRightMotorConfiguration.pid = kPIDConfiguration;
        kRightMotorConfiguration.neutralMode = NeutralModeValue.Coast;
        kRightMotorConfiguration.supplyCurrentLimit = 40.0;
    }
}

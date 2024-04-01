package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SoftLimitsConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceType;
import frc.robot.Constants;

public class ArmConstants {
    public static final double kAbsoluteEncoderOffset = Units.degreesToRotations(313.2 - 180.0); //312.7?
    public static final boolean kAbsoluteEncoderInverted = false;

    public static final double kPlanetaryReduction = 4.0 * 5.0;
    public static final double kGearRatio = kPlanetaryReduction * (64.0 / 16.0) * (64.0 / 36.0);

    public static final double kPadding = 0.5; // degrees
    public static final double kRoughPadding = 5.0; //degrees
    public static final double kCruiseVelocity = 200.0; // degrees/sec
    public static final double kTimeToCruise = 0.5; // sec

    public static final double kMinAngle = 155.0; // degrees
    public static final double kMaxAngle = 300.0; // degrees

    public static final double kArmP = 10.0;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;

    public static final double kArmS = 0.0;
    public static final double kArmG = 0.3;
    public static final double kArmV = 0.0;
    public static final double kArmA = 0.0;

    public static final SoftLimitsConfiguration kLimitConfiguration = new SoftLimitsConfiguration();
    static {
        kLimitConfiguration.kUpperLimit = Units.degreesToRotations(kMaxAngle);
        kLimitConfiguration.kLowerLimit = Units.degreesToRotations(kMinAngle);
    }

    public static final PIDConfiguration kPIDConfiguration = new PIDConfiguration();
    static {
        kPIDConfiguration.kP = kArmP;
        kPIDConfiguration.kI = kArmI;
        kPIDConfiguration.kD = kArmD;
    }

    public static final Configuration kLeaderMotorConfiguration = new Configuration();
    static {
        kLeaderMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, Constants.RobotMap.kArmLeaderMotor);

        kLeaderMotorConfiguration.pid = kPIDConfiguration;
        kLeaderMotorConfiguration.limits = kLimitConfiguration;

        kLeaderMotorConfiguration.kVoltageCompensation = 12.0;
        kLeaderMotorConfiguration.kShouldInvert = false;
        kLeaderMotorConfiguration.kIdleMode = IdleMode.kBrake;
        kLeaderMotorConfiguration.kOpenLoopRampRate = 1.0;
        kLeaderMotorConfiguration.kClosedLoopRampRate = 0.5;
        kLeaderMotorConfiguration.kSmartCurrentLimit = 30.0;
    }

    public static final Configuration kFollowerMotorConfiguration = new Configuration();
    static {
        kFollowerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, Constants.RobotMap.kArmFollowerMotor);

        kFollowerMotorConfiguration.kVoltageCompensation = 12.0;
        kFollowerMotorConfiguration.kIdleMode = IdleMode.kBrake;
        kFollowerMotorConfiguration.kSmartCurrentLimit = 30.0;
    }

    public static double constrainDegrees(double degrees) {
        return MathUtil.clamp(degrees, kMinAngle, kMaxAngle);
    }
}
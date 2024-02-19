package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.MathUtil;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.SoftLimitsConfiguration;
import frc.lib.team5557.factory.SparkMaxFactory.Configuration;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceId.CANDeviceType;
import frc.robot.Constants;

public class ArmConstants {
    public static final double kGearReduction = 36.0;
    public static final double kFirstSprocketTeethCount = 16.0; //teeth
    public static final double kSecondSprocketTeethCount = 48.0; //teeth
    public static final double kRotationsPerDegree = kGearReduction * (kSecondSprocketTeethCount / kFirstSprocketTeethCount) / 360.0;

    public static final double kEncoderHomePosition = 279.5; //degrees
    public static final double kPadding = 1.0; // degrees
    public static final double kCruiseVelocity = 150.0; // degrees/sec
    public static final double kTimeToCruise = 0.1; // sec

    public static final double kHomeVoltage = 2.0;
    public static final double kHomeAmpsThreshold = 15.0;

    public static final double kMinAngle = 170.0; //degrees
    public static final double kMaxAngle = 270.0; //degrees

    public static final double kArmkP = 0.08;
    public static final double kArmkI = 0.0;
    public static final double kArmkD = 0.0;

    public static final double kArmkS = 0.0;
    public static final double kArmkG = 0.0;
    public static final double kArmkA = 0.0;
    public static final double kArmkV = 0.0;
    
    public static final SoftLimitsConfiguration kLimitConfiguration = new SoftLimitsConfiguration();
    static {
        kLimitConfiguration.kUpperLimit = degreesToRotations(kMaxAngle);
        kLimitConfiguration.kLowerLimit = degreesToRotations(kMinAngle);
    }

    public static final PIDConfiguration kPIDConfiguration = new PIDConfiguration();
    static {
        kPIDConfiguration.kP = kArmkP;
        kPIDConfiguration.kI = kArmkI;
        kPIDConfiguration.kD = kArmkD;
        kPIDConfiguration.kF = 0.0;
        kPIDConfiguration.kTolerance = degreesToRotations(kPadding);
    }

    public static final Configuration kArmLeaderMotorConfiguration = new Configuration();
    static {
        kArmLeaderMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, Constants.RobotMap.kArmLeaderMotor);
        
        kArmLeaderMotorConfiguration.pid = kPIDConfiguration;
        kArmLeaderMotorConfiguration.limits = kLimitConfiguration;

        kArmLeaderMotorConfiguration.kVoltageCompensation = 12.0;
        kArmLeaderMotorConfiguration.kShouldInvert = true;
        kArmLeaderMotorConfiguration.kIdleMode = IdleMode.kCoast;
        kArmLeaderMotorConfiguration.kOpenLoopRampRate = 1.0;
        kArmLeaderMotorConfiguration.kClosedLoopRampRate = 0.5;
        kArmLeaderMotorConfiguration.kSmartCurrentLimit = 30.0;
    }

    public static final Configuration kArmFollowerMotorConfiguration = new Configuration();
    static {
        kArmFollowerMotorConfiguration.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, Constants.RobotMap.kArmFollowerMotor);
        
        kArmLeaderMotorConfiguration.pid = kPIDConfiguration;
        kArmLeaderMotorConfiguration.limits = kLimitConfiguration;
        
        kArmFollowerMotorConfiguration.kShouldInvert = false;

        kArmLeaderMotorConfiguration.kVoltageCompensation = 12.0;
        kArmLeaderMotorConfiguration.kIdleMode = IdleMode.kCoast;
        kArmLeaderMotorConfiguration.kOpenLoopRampRate = 1.0;
        kArmLeaderMotorConfiguration.kClosedLoopRampRate = 0.5;
        kArmLeaderMotorConfiguration.kSmartCurrentLimit = 30.0;
        
    }

    public static double rotationsToDegrees(double rotations) {
        return rotations / kRotationsPerDegree;
    }

    public static double degreesToRotations(double degrees) {
        return degrees * kRotationsPerDegree;
    }

    public static double constrainDegrees(double degrees) {
        return MathUtil.clamp(degrees, kMinAngle, kMaxAngle);
    }
}
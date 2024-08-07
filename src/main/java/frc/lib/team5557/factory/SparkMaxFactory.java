package frc.lib.team5557.factory;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import frc.lib.team5557.util.CANDeviceType;
import frc.lib.team5557.util.CANDeviceFinder;
import frc.lib.team5557.util.CANDeviceId;

public class SparkMaxFactory {
	public static final int configCANTimeout = 400;
	public static final int configCount = 1;

	private static final CANDeviceFinder can = new CANDeviceFinder();

	public static class Configuration {
		public String label;
		public CANDeviceId canID;

		public PIDConfiguration pid;
		public SoftLimitsConfiguration limits = null;

		public IdleMode kIdleMode = IdleMode.kCoast;
		public boolean kShouldInvert = false;

		public double kSmartCurrentLimit = Double.NaN;
		public double kVoltageCompensation = Double.NaN;
		public double kOpenLoopRampRate = 0.0;
		public double kClosedLoopRampRate = 0.0;
	}

	public static class PIDConfiguration {
		public int kSlot = 0;
		public double kP = 0.0;
		public double kI = 0.0;
		public double kD = 0.0;
		public double kF = 0.0;
		public double kTolerance = 0.0;
		public double kMaxEffort = 12.0;
	}

	public static class SoftLimitsConfiguration {
		public double kUpperLimit = Double.NaN;
		public double kLowerLimit = Double.NaN;
	}

	public static CANSparkMax createNEO(int id, Configuration config) {
		config.canID = new CANDeviceId(CANDeviceType.SPARK_MAX, id);
		return createNEO(config);
	}

	public static CANSparkMax createNEO(Configuration config) {
		can.isDevicePresent(config.canID.getDeviceType(), config.canID.getDeviceNumber(), config.label);

		CANSparkMax sparkMax = new CANSparkMax(config.canID.getDeviceNumber(), MotorType.kBrushless);
		SparkPIDController pid = sparkMax.getPIDController();
		RelativeEncoder encoder = sparkMax.getEncoder();

		BurnManager.restoreFactoryDefaults(sparkMax);

		sparkMax.setCANTimeout(configCANTimeout);

		for (int i = 0; i < configCount; i++) {
			if (!Double.isNaN(config.kSmartCurrentLimit)) {
				sparkMax.setSmartCurrentLimit((int) config.kSmartCurrentLimit);
			}
			if (!Double.isNaN(config.kVoltageCompensation)) {
				sparkMax.enableVoltageCompensation(config.kVoltageCompensation);
			}

			sparkMax.setIdleMode(config.kIdleMode);
			sparkMax.setInverted(config.kShouldInvert);
			sparkMax.setOpenLoopRampRate(config.kOpenLoopRampRate);
			sparkMax.setClosedLoopRampRate(config.kClosedLoopRampRate);

			encoder.setPositionConversionFactor(1.0);
			encoder.setVelocityConversionFactor(1.0);
			encoder.setMeasurementPeriod(10);
			encoder.setAverageDepth(2);

			if (config.pid != null) {
				pid.setP(config.pid.kP, 0);
				pid.setI(config.pid.kI, 0);
				pid.setD(config.pid.kD, 0);
				pid.setFF(config.pid.kF, 0);

				pid.setOutputRange(-config.pid.kMaxEffort, config.pid.kMaxEffort, 0);
				pid.setIZone(config.pid.kTolerance * 2, 0);
				pid.setIMaxAccum(5.0, 0);
			}

			if (config.limits != null) {
				configSoftLimits(sparkMax, config.limits);
			}
		}
		sparkMax.setCANTimeout(0);

		return sparkMax;
	}

	public static void configFramesDefault(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
	}

	public static void configFramesLeader(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
	}

	public static void configFramesFollower(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 65535);
	}

	public static void configFramesPositionBoost(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 10);
	}

	public static void configFramesAbsoluteEncoderBoost(CANSparkMax sparkMax) {
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 20);
		sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 20);
	}

	public static void configSoftLimits(CANSparkMax sparkMax, SoftLimitsConfiguration config) {
		if (!Double.isNaN(config.kLowerLimit)) {
			sparkMax.setSoftLimit(SoftLimitDirection.kReverse, (float) config.kLowerLimit);
			sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, true);
		} else {
			sparkMax.enableSoftLimit(SoftLimitDirection.kReverse, false);
		}

		if (!Double.isNaN(config.kUpperLimit)) {
			sparkMax.setSoftLimit(SoftLimitDirection.kForward, (float) config.kUpperLimit);
			sparkMax.enableSoftLimit(SoftLimitDirection.kForward, true);
		} else {
			sparkMax.enableSoftLimit(SoftLimitDirection.kForward, false);
		}
	}
}

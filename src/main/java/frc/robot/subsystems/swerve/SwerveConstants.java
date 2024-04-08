package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team5557.factory.TalonFactory;
import frc.lib.team5557.factory.SparkMaxFactory.PIDConfiguration;
import frc.robot.subsystems.swerve.util.KinematicLimits;
import frc.robot.subsystems.swerve.util.ModuleLimits;

public class SwerveConstants {

	public static final String kSubsystemName = "Swerve";

	// Physical Constants
	public static final double kChassisLength = Units.inchesToMeters(28.765);
	public static final double kChassisWidth = Units.inchesToMeters(26.0);
	public static final double kDrivetrainLength = Units.inchesToMeters(23.0);

	public static final double kTrackWidth = kChassisWidth - Units.inchesToMeters(2 * 2.625);
	public static final double kWheelBase = kDrivetrainLength - Units.inchesToMeters(2 * 2.625);
	public static final double kChassisToBullBar = kChassisLength - kDrivetrainLength;
	public static final double kTrueChassisCenterOffset = kChassisToBullBar / 2;

	public static final double kAngleGearReduction = 150.0 / 7.0;
	public static final double kDriveGearReduction = (50.0 / 16.0) * (16.0 / 28.0) * (45.0 / 15.0);
	public static final double kWheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

	public static final double kKrakenFreeSpeed = 6000;
	public static final double kKrakenFreeSpeedFOC = 5800;

	/* Swerve Profiling Values */
	public static final double kTheoreticalMaxSpeed = kWheelCircumference * ((kKrakenFreeSpeed/60.0) / kDriveGearReduction); // meters per second
	public static final double kTheoreticalMaxAcceleration = 7.090; // m/s^2
	public static final double kTheoreticalMaxOmega = 11.5; // radians per second

	public static final double kTrueMaxSpeed = kTheoreticalMaxSpeed * 0.8; // Max out at 85% to make sure speeds are
																			// attainable (4.6 mps)
	public static final double kTrueMaxAcceleration = kTheoreticalMaxAcceleration * 0.8;
	public static final double kTrueMaxOmega = kTheoreticalMaxOmega * 0.85;

	// Angle Encoder Constants
	public static final boolean kAbsoluteEncoderInverted = true;
	public static final int kAbsoluteResetIterations = 200;
	public static final double kAbsoluteResetMaxOmega = Units.degreesToRadians(1.0); // must rotate at less than a degree per second

	/* ANGLE PID */
	public static final double kAnglekP = 0.3; // error(rotations) * kP = volts -> kp = 0.6 = 12.0 volts / 20 rotations
	public static final double kAnglekI = 0.0;
	public static final double kAnglekD = 0.0;

	/* DRIVE PID */
	public static final double kDrivekP = 0.0; // FIXME: error(rpm) * kP = volts -> kp = 0.6 = 12.0 volts / 5000 rpm
	public static final double kDrivekI = 0.0;
	public static final double kDrivekD = 0.0;

	public static final double kDrivekS = 0.16083;
	public static final double kDrivekV = 1.88797;//12.0 / kTheoreticalMaxSpeed;
	public static final double kDrivekA = 0.0;

	/* MOTION PLANNER PID */
	public static final double kTranslationkP = 5.0; // error(meters) * kP = velocity m/s -> kp = velocity / error -->
														// 4.5
	public static final double kTranslationkI = 0.0;
	public static final double kTranslationkD = 0.0;

	public static final double kRotationkP = 4.0;
	public static final double kRotationkI = 0.0;
	public static final double kRotationkD = 0.0;

	public static final double kSnapMaxOmega = kTrueMaxOmega * 0.65;
	public static final double kSnapMaxAlpha = kSnapMaxOmega / 0.15;

	// MODULE LIMITS
	public static final ModuleLimits kAutoModuleLimits = new ModuleLimits(kTrueMaxSpeed, kTheoreticalMaxAcceleration,
			Units.degreesToRadians(1080.0));
	public static final ModuleLimits kTeleopModuleLimits = new ModuleLimits(kTheoreticalMaxSpeed, kTrueMaxSpeed * 5,
			Units.degreesToRadians(1080.0));

	// KINEMATIC LIMITS
	public static final KinematicLimits kUncappedLimits = new KinematicLimits(kTheoreticalMaxSpeed, Double.MAX_VALUE,
			kTheoreticalMaxOmega, Double.MAX_VALUE);
	public static final KinematicLimits kDrivingLimits = new KinematicLimits(kTheoreticalMaxSpeed, kTheoreticalMaxSpeed * 5.0,
			kTrueMaxOmega * 0.65, Double.MAX_VALUE);
	public static final KinematicLimits kBrownOutLimits = new KinematicLimits(kTheoreticalMaxSpeed * 0.5, kTheoreticalMaxSpeed * 0.5,
			kTrueMaxOmega * 0.65, Double.MAX_VALUE);
	public static final KinematicLimits kAutoLimits = new KinematicLimits(kTrueMaxSpeed, Double.MAX_VALUE,
			kTrueMaxOmega,
			Double.MAX_VALUE);
	public static final KinematicLimits kShootingLimits = new KinematicLimits(kTrueMaxSpeed * 0.5, 3.0,
			kTrueMaxOmega * 0.5, Double.MAX_VALUE);

	// DRIVE MOTOR CONFIGURATION
	public static TalonFactory.PIDConfiguration kDrivePIDConfiguration = new TalonFactory.PIDConfiguration();
	static {
		kDrivePIDConfiguration.kP = kDrivekP;
		kDrivePIDConfiguration.kI = kDrivekI;
		kDrivePIDConfiguration.kD = kDrivekD;
	}

	public static TalonFactory.Configuration kDriveMotorConfiguration = new TalonFactory.Configuration();
	static {
		kDriveMotorConfiguration.label = "Drive Motor";
		kDriveMotorConfiguration.setInverted = true;
		kDriveMotorConfiguration.pid = kDrivePIDConfiguration;
		kDriveMotorConfiguration.neutralMode = NeutralModeValue.Brake;
		kDriveMotorConfiguration.supplyCurrentLimit = 60.0;
		kDriveMotorConfiguration.statorCurrentLimit = 60.0;
	}

	// ANGLE MOTOR CONFIGURATION
	public static PIDConfiguration kAnglePIDConfiguration = new PIDConfiguration();
	static {
		kAnglePIDConfiguration.kP = kAnglekP;
		kAnglePIDConfiguration.kI = kAnglekI;
		kAnglePIDConfiguration.kD = kAnglekD;
	}

	public static SparkMaxFactory.Configuration kAngleMotorConfiguration = new SparkMaxFactory.Configuration();
	static {
		kAngleMotorConfiguration.pid = kAnglePIDConfiguration;

		kAngleMotorConfiguration.kShouldInvert = true;
		kAngleMotorConfiguration.kVoltageCompensation = 12.0;
		kAngleMotorConfiguration.kSmartCurrentLimit = 30.0;
		kAngleMotorConfiguration.kIdleMode = IdleMode.kBrake;
	}

	public static final Translation2d[] kSwerveModuleLocations = {
			new Translation2d((kWheelBase / 2.0) - kTrueChassisCenterOffset, kTrackWidth / 2.0),
			new Translation2d((kWheelBase / 2.0) - kTrueChassisCenterOffset, -kTrackWidth / 2.0),
			new Translation2d((-kWheelBase / 2.0) - kTrueChassisCenterOffset, kTrackWidth / 2.0),
			new Translation2d((-kWheelBase / 2.0) - kTrueChassisCenterOffset, -kTrackWidth / 2.0)
	};

	public static double kDriveBaseRadius = kSwerveModuleLocations[3].getNorm();

	public static final SwerveModuleState[] kXOutSwerveModuleStates = {
			new SwerveModuleState(0.0, Rotation2d.fromDegrees(45)),
			new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
			new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45)),
			new SwerveModuleState(0.0, Rotation2d.fromDegrees(45))
	};

	/**
	 * @param counts    NEO Rotations
	 * @param gearRatio Gear Ratio between NEO and Mechanism
	 * @return Degrees of Rotation of Mechanism
	 */
	public static double rotationsToDegrees(double rotations, double gearRatio) {
		return rotations * (360.0 / gearRatio);
	}

	/**
	 * @param counts        NEO rotations
	 * @param circumference Circumference of Wheel
	 * @param gearRatio     Gear Ratio between Motor and Mechanism (set to 1 for
	 *                      Falcon RPM)
	 * @return Falcon Velocity Counts
	 */
	public static double rotationsToMeters(double rotations, double circumference, double gearRatio) {
		double wheelRotations = rotations / gearRatio;
		return (wheelRotations * circumference);
	}

	/**
	 * @param counts        NEO rotations
	 * @param circumference Circumference of Wheel
	 * @param gearRatio     Gear Ratio between Motor and Mechanism (set to 1 for
	 *                      Falcon RPM)
	 * @return Falcon Velocity Counts
	 */
	public static double metersToRotations(double meters, double circumference, double gearRatio) {
		double wheelRotations = meters / circumference;
		return (wheelRotations * gearRatio);
	}

	public static double radiansToRotations(double radians, double gearRatio) {
		return (radians / (2 * Math.PI)) * gearRatio;
	}

	public static double rotationsToRadians(double rotations, double gearRatio) {
		return (rotations * (2 * Math.PI)) / gearRatio;
	}
}

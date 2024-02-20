package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.auto.SystemsCheckManager.SwerveModuleSystemCheckRequest;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.util.KinematicLimits;
import frc.robot.subsystems.swerve.util.SwerveSetpoint;
import frc.robot.subsystems.swerve.util.SwerveSetpointGenerator;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Swerve extends SubsystemBase {
	private final Module[] mModules = new Module[4]; // FL, FR, BL, BR

	private final GyroIO mGyroIO;
	private final GyroIOInputs mGyroInputs = new GyroIOInputs();

	private ControlMode mControlMode = ControlMode.VELOCITY;
	private KinematicLimits mKinematicLimits = kUncappedLimits;
	private SwerveModuleSystemCheckRequest m_systemCheckState = SwerveModuleSystemCheckRequest.DO_NOTHING;

	private ChassisSpeeds mDesiredSpeeds = new ChassisSpeeds();
	private Twist2d mFieldVelocity = new Twist2d();

	public static final SwerveDriveKinematics mKinematics = new SwerveDriveKinematics(kSwerveModuleLocations);
	private SwerveSetpointGenerator mSetpointGenerator = new SwerveSetpointGenerator(mKinematics,
			kSwerveModuleLocations);
	private SwerveSetpoint mSwerveSetpoint = new SwerveSetpoint(
			mDesiredSpeeds,
			new SwerveModuleState[] {
					new SwerveModuleState(),
					new SwerveModuleState(),
					new SwerveModuleState(),
					new SwerveModuleState()
			});

	private int mSystemCheckModuleNumber = 0;
	private double mCharacterizationVolts = 0.0;

	public enum ControlMode {
		X_OUT,
		OPEN_LOOP,
		VELOCITY,
		PATH_FOLLOWING,
		CHARACTERIZATION,
		SYSTEMS_CHECK
	}

	public Swerve(
			GyroIO gyroIO,
			ModuleIO flModuleIO,
			ModuleIO frModuleIO,
			ModuleIO blModuleIO,
			ModuleIO brModuleIO) {
		System.out.println("[Init] Creating Swerve");
		this.mGyroIO = gyroIO;
		mModules[0] = new Module(flModuleIO, 0);
		mModules[1] = new Module(frModuleIO, 1);
		mModules[2] = new Module(blModuleIO, 2);
		mModules[3] = new Module(brModuleIO, 3);
    SmartDashboard.putData("Swerve Viewable", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", () -> m_modules[0].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_modules[0].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Front Right Angle", () -> m_modules[1].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_modules[1].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Left Angle", () -> m_modules[2].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_modules[2].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Right Angle", () -> m_modules[3].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_modules[3].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Robot Angle",
            () -> (RobotStateEstimator.isRedAlliance)
                ? getGyroYaw().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians()
                : getGyroYaw().getRadians(),
            null);
      }
    });

/**
    AutoBuilder.configureHolonomic(
            RobotStateEstimator.getInstance().getPose(), 
            this::resetPose(), 
            this::getRobotRelativeSpeeds, 
            this::drive, 
            new HolonomicPathFollowerConfig( 
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                    4.5, // Max module speed, in m/s
                    0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                    new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this 
    ); */
  }

		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve");
		shuffleboardTab
				.addNumber("Heading", () -> Util.truncate(getGyroYaw().getDegrees(), 2))
				.withWidget(BuiltInWidgets.kGyro);
		shuffleboardTab
				.addNumber(
						"Velocity",
						() -> Util.truncate(Math.hypot(getFieldVelocity().dx, getFieldVelocity().dy), 2))
				.withWidget(BuiltInWidgets.kGraph);

		shuffleboardTab
				.addNumber("Velocity Kinematic Limit", () -> getKinematicLimit().maxLinearVelocity())
				.withWidget(BuiltInWidgets.kNumberBar);
		shuffleboardTab.addString("Control Mode", () -> getControlMode().name());
		shuffleboardTab.addString(
				"Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "NONE");

		SmartDashboard.putData("Swerve Viewable", new Sendable() {
			@Override
			public void initSendable(SendableBuilder builder) {
				builder.setSmartDashboardType("SwerveDrive");
				builder.addDoubleProperty("Front Left Angle", () -> mModules[0].getState().angle.getRadians(), null);
				builder.addDoubleProperty("Front Left Velocity", () -> mModules[0].getState().speedMetersPerSecond,
						null);
				builder.addDoubleProperty("Front Right Angle", () -> mModules[1].getState().angle.getRadians(), null);
				builder.addDoubleProperty("Front Right Velocity", () -> mModules[1].getState().speedMetersPerSecond,
						null);
				builder.addDoubleProperty("Back Left Angle", () -> mModules[2].getState().angle.getRadians(), null);
				builder.addDoubleProperty("Back Left Velocity", () -> mModules[2].getState().speedMetersPerSecond,
						null);
				builder.addDoubleProperty("Back Right Angle", () -> mModules[3].getState().angle.getRadians(), null);
				builder.addDoubleProperty("Back Right Velocity", () -> mModules[3].getState().speedMetersPerSecond,
						null);
				builder.addDoubleProperty("Robot Angle",
						() -> (AllianceFlipUtil.shouldFlip())
								? getGyroYaw().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians()
								: getGyroYaw().getRadians(),
						null);
			}
		});
	}

	@Override
	public void periodic() {
		mGyroIO.updateInputs(mGyroInputs);
		Logger.processInputs(kSubsystemName + "/Gyro", mGyroInputs);
		for (var module : mModules) {
			module.updateAndProcessInputs();
		}

		// Run modules
		if (DriverStation.isDisabled()) {
			// Stop moving while disabled
			for (var module : mModules) {
				module.stop();
			}

			// Clear setpoint logs
			Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
			Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});

		} else if (mControlMode == ControlMode.X_OUT) {
			for (int i = 0; i < mModules.length; i++) {
				mModules[i].runSetpoint(kXOutSwerveModuleStates[i], true, true);
			}
		} else if (mControlMode == ControlMode.CHARACTERIZATION) {
			// Run in characterization mode
			for (var module : mModules) {
				module.setVoltageForCharacterization(mCharacterizationVolts);
			}

			// Clear setpoint logs
			Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
			Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});

		} else if (mControlMode == ControlMode.SYSTEMS_CHECK) {
			for (int i = 0; i < mModules.length; i++) {
				if (i == mSystemCheckModuleNumber) {
					if (m_systemCheckState.isSetpointCheck()) {
						mModules[i].runSetpoint(m_systemCheckState.getSwerveModuleState(), true, true);
					} else {
						mModules[i].setVoltageForAzimuthCheck(m_systemCheckState.getSteerVoltage());
					}
				} else {
					mModules[i].stop();
				}
			}
			/* FIX THIS */

			// Clear setpoint logs
			Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
			Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});
		} else {
			// Calculate module setpoints
			var setpointTwist = new Pose2d()
					.log(
							new Pose2d(
									mDesiredSpeeds.vxMetersPerSecond * Robot.defaultPeriodSecs,
									mDesiredSpeeds.vyMetersPerSecond * Robot.defaultPeriodSecs,
									new Rotation2d(
											mDesiredSpeeds.omegaRadiansPerSecond * Robot.defaultPeriodSecs * 4)));

			mDesiredSpeeds = new ChassisSpeeds(
					setpointTwist.dx / Robot.defaultPeriodSecs,
					setpointTwist.dy / Robot.defaultPeriodSecs,
					mDesiredSpeeds.omegaRadiansPerSecond);

			mSwerveSetpoint = mSetpointGenerator.generateSetpoint(
					kModuleLimits, mKinematicLimits, mSwerveSetpoint, mDesiredSpeeds, Robot.defaultPeriodSecs);

			// m_swerveSetpoint.moduleStates =
			// m_kinematics.toSwerveModuleStates(adjustedSpeeds);

			// Send setpoints to modules
			SwerveModuleState[] optimizedStates = new SwerveModuleState[4];

			if (mControlMode == ControlMode.PATH_FOLLOWING) {
				for (int i = 0; i < 4; i++) {
					optimizedStates[i] = mModules[i].runSetpoint(mSwerveSetpoint.moduleStates()[i], false, false);
				}
			} else {
				for (int i = 0; i < 4; i++) {
					optimizedStates[i] = mModules[i].runSetpoint(mSwerveSetpoint.moduleStates()[i], true, false);
				}
			}

			// Log setpoint states
			Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", mSwerveSetpoint.moduleStates());
			Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", optimizedStates);
		}

		// Log measured states
		SwerveModuleState[] measuredStates = new SwerveModuleState[4];
		for (int i = 0; i < 4; i++) {
			measuredStates[i] = mModules[i].getState();
		}

		Logger.recordOutput(kSubsystemName + "/ModuleStates/Measured", measuredStates);
		// Get measured positions
		SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
			measuredPositions[i] = mModules[i].getPosition();
		}

		// Update Pose Estimator
		if (mGyroInputs.connected) {
			RobotStateEstimator.getInstance().addDriveData(getGyroYaw(), measuredPositions);
		} else {
			RobotStateEstimator.getInstance().addDriveData(measuredPositions);
		}

		// Update field velocity
		ChassisSpeeds chassisSpeeds = mKinematics.toChassisSpeeds(measuredStates);
		Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
				chassisSpeeds.vyMetersPerSecond)
				.rotateBy(getGyroYaw());
		mFieldVelocity = new Twist2d(
				linearFieldVelocity.getX(),
				linearFieldVelocity.getY(),
				mGyroInputs.connected
						? mGyroInputs.yawVelocityRadPerSec
						: chassisSpeeds.omegaRadiansPerSecond);

		Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VX", mFieldVelocity.dx);
		Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VY", mFieldVelocity.dy);
		Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VTHETA", mFieldVelocity.dtheta);
		Logger.recordOutput(kSubsystemName + "/ControlMode", mControlMode.toString());
		Logger.recordOutput(kSubsystemName + "/KinematicLimits", mKinematicLimits.toString());
	}

	public void drive(ChassisSpeeds desSpeed, ControlMode desMode) {
		mDesiredSpeeds = desSpeed;
		mControlMode = desMode;
	}

	public void driveOpenLoop(ChassisSpeeds desSpeed) {
		this.drive(desSpeed, ControlMode.OPEN_LOOP);
	}

	public void driveVelocity(ChassisSpeeds desSpeed) {
		this.drive(desSpeed, ControlMode.VELOCITY);
	}

	public void drivePath(ChassisSpeeds desSpeed) {
		if (mControlMode != ControlMode.PATH_FOLLOWING) {
			setKinematicLimits(kAutoLimits);
		}
		this.drive(desSpeed, ControlMode.PATH_FOLLOWING);
	}

	public void runModuleCheck(int moduleNumber, SwerveModuleSystemCheckRequest state) {
		if (mControlMode != ControlMode.SYSTEMS_CHECK) {
			mControlMode = ControlMode.SYSTEMS_CHECK;
		}
		m_systemCheckState = state;
		mSystemCheckModuleNumber = moduleNumber;
	}

	public void stop() {
		this.drive(new ChassisSpeeds(), ControlMode.OPEN_LOOP);
	}

	public void stopWithX() {
		this.drive(new ChassisSpeeds(), ControlMode.X_OUT);
	}

	public Rotation2d getGyroYaw() {
		return mGyroInputs.connected
				? mGyroInputs.yawPosition
				: RobotStateEstimator.getInstance().getPose().getRotation();
	}

	public SwerveSetpoint getSwerveSetpoint() {
		return mSwerveSetpoint;
	}

	public Twist2d getFieldVelocity() {
		return mFieldVelocity;
	}

	public KinematicLimits getKinematicLimit() {
		return mKinematicLimits;
	}

	public ControlMode getControlMode() {
		return mControlMode;
	}

	public boolean isUnderKinematicLimit(KinematicLimits limits) {
		return Math.hypot(getFieldVelocity().dx, getFieldVelocity().dy) < limits.maxLinearVelocity();
	}

	public void setKinematicLimits(KinematicLimits limits) {
		mKinematicLimits = limits;
	}
}

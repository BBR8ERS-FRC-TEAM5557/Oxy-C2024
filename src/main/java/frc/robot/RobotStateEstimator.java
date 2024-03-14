package frc.robot;

import java.util.List;
import java.util.Map;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.team6328.LoggedTunableNumber;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;

public class RobotStateEstimator extends VirtualSubsystem {

	public record OdometryObservation(
			SwerveDriveWheelPositions wheelPositions, Rotation2d gyroAngle, double timestamp) {
	}

	public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
	}

	public record AimingParameters(
			Rotation2d driveHeading, Rotation2d armAngle, double driveFeedVelocity) {
	}

	private static RobotStateEstimator mInstance = null;

	public static RobotStateEstimator getInstance() {
		if (mInstance == null) {
			System.out.println("[Init] Creating RobotStateEstimator");
			mInstance = new RobotStateEstimator();
		}
		return mInstance;
	}

	private static GenericEntry mShotCompensationDegrees;
	private static final LoggedTunableNumber lookahead = new LoggedTunableNumber("RobotState/lookaheadS", 0.0);
	private static final double kMeasurementOffset = (SwerveConstants.kChassisLength / 2.0)
			+ Units.inchesToMeters(24.0);

	/**
	 * Arm angle (single side flywheels) look up table key: meters, values: degrees
	 */
	// STILL NEEDS TO BE FILLED IN
	private static final InterpolatingDoubleTreeMap armAngleMapSingle = new InterpolatingDoubleTreeMap();
	static {
		armAngleMapSingle.put(Units.inchesToMeters(0.0), 157.0); // lower limit

		armAngleMapSingle.put(Units.inchesToMeters(0.0) + kMeasurementOffset, 157.0); // from subwoofer
		//armAngleMapSingle.put(Units.inchesToMeters(24.0) + kMeasurementOffset, 164.0); // 165.5 true
		armAngleMapSingle.put(Units.inchesToMeters(48.0) + kMeasurementOffset, 174.0); // 172.5 true
		//armAngleMapSingle.put(Units.inchesToMeters(72.0) + kMeasurementOffset, 177.0); // 178.5 true
		armAngleMapSingle.put(Units.inchesToMeters(96.0) + kMeasurementOffset, 182.0); // 182.5 true
		//armAngleMapSingle.put(Units.inchesToMeters(120.0) + kMeasurementOffset, 181.5); // 182.5 true
		armAngleMapSingle.put(Units.inchesToMeters(144.0) + kMeasurementOffset, 185.0); // 182.5 true
		//armAngleMapSingle.put(Units.inchesToMeters(168.0) + kMeasurementOffset, 184.0); // 182.5 true

		armAngleMapSingle.put(Double.MAX_VALUE, 190.0); // upper limit
	}

	/**
	 * Arm angle (double side flywheels) look up table key: meters, values: degrees
	 */
	// THIS ONE WAS DONE PROPERLY
	private static final InterpolatingDoubleTreeMap armAngleMapDouble = new InterpolatingDoubleTreeMap();
	static {
		armAngleMapDouble.put(Units.inchesToMeters(0.0), 157.0); // lower limit

		armAngleMapDouble.put(Units.inchesToMeters(0.0) + kMeasurementOffset, 160.0); // from subwoofer
		armAngleMapDouble.put(Units.inchesToMeters(24.0) + kMeasurementOffset, 166.0); // 165.5 true
		armAngleMapDouble.put(Units.inchesToMeters(48.0) + kMeasurementOffset, 172.0); // 172.5 true
		armAngleMapDouble.put(Units.inchesToMeters(72.0) + kMeasurementOffset, 179.0); // 178.5 true
		armAngleMapDouble.put(Units.inchesToMeters(96.0) + kMeasurementOffset, 183.0); // 182.5 true
		armAngleMapDouble.put(Units.inchesToMeters(120.0) + kMeasurementOffset, 185.5); // 182.5 true

		armAngleMapDouble.put(Double.MAX_VALUE, 190.0); // upper limit
	}

	private final SwerveDrivePoseEstimator mPoseEstimator;
	private SwerveDriveWheelPositions mLastModulePositions = new SwerveDriveWheelPositions(
			new SwerveModulePosition[] {
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition(),
					new SwerveModulePosition()
			});

	/**
	 * Cached latest aiming parameters. Calculated in {@code getAimingParameters()}
	 */
	private AimingParameters mLatestParameters = null;
	private Twist2d mRobotVelocity = new Twist2d();
	private final Field2d mField2d = new Field2d();

	private RobotStateEstimator() {
		mPoseEstimator = new SwerveDrivePoseEstimator(
				Swerve.mKinematics, new Rotation2d(), mLastModulePositions.positions, new Pose2d());

		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
		shuffleboardTab.add(mField2d);
		mShotCompensationDegrees = shuffleboardTab
				.add("shotCompensationDegrees", 0.0).withWidget(BuiltInWidgets.kNumberSlider)
				.withProperties(Map.of("min", -3.0, "max", 3.0, "Block increment", 0.25))
				.getEntry();
	}

	@Override
	public void periodic() {
		clampPoseToField();
		updateFieldWidget();
	}

	/** Add odometry observation */
	public void addOdometryObservation(OdometryObservation observation) {
		mLatestParameters = null;
		var twist = Swerve.mKinematics.toTwist2d(mLastModulePositions, observation.wheelPositions());
		var derivedGyroAngle = Rotation2d.fromRadians(getEstimatedPose().getRotation().getRadians() + twist.dtheta);
		mLastModulePositions = observation.wheelPositions;

		// Check gyro connected
		if (observation.gyroAngle != null) {
			mPoseEstimator.update(observation.gyroAngle, observation.wheelPositions);
		} else {
			mPoseEstimator.update(derivedGyroAngle, observation.wheelPositions);
		}
	}

	public void addVisionObservation(List<VisionObservation> visionData) {
		for (var update : visionData) {
			mPoseEstimator.addVisionMeasurement(update.visionPose, update.timestamp, update.stdDevs);
		}
	}

	public void addVelocityData(Twist2d robotVelocity) {
		mLatestParameters = null;
		this.mRobotVelocity = robotVelocity;
	}

	public AimingParameters getAimingParameters() {
		if (mLatestParameters != null) {
			// Cache previously calculated aiming parameters. Cache is invalidated whenever
			// new
			// observations are added.
			return mLatestParameters;
		}
		Transform2d fieldToTarget = GeometryUtil
				.translationToTransform(AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening)
						.toTranslation2d());
		Pose2d fieldToPredictedVehicle = getPredictedPose(lookahead.get(), lookahead.get());
		Pose2d fieldToPredictedVehicleFixed = new Pose2d(fieldToPredictedVehicle.getTranslation(), new Rotation2d());

		Translation2d predictedVehicleToTargetTranslation = GeometryUtil.inverse(fieldToPredictedVehicle)
				.transformBy(fieldToTarget)
				.getTranslation();
		Translation2d predictedVehicleFixedToTargetTranslation = GeometryUtil.inverse(fieldToPredictedVehicleFixed)
				.transformBy(fieldToTarget).getTranslation();

		Rotation2d vehicleToGoalDirection = predictedVehicleToTargetTranslation.getAngle();

		Rotation2d targetVehicleDirection = predictedVehicleFixedToTargetTranslation.getAngle();
		double targetDistance = predictedVehicleToTargetTranslation.getNorm();

		double feedVelocity = mRobotVelocity.dx * vehicleToGoalDirection.getSin() / targetDistance
				- mRobotVelocity.dy * vehicleToGoalDirection.getCos() / targetDistance;

		mLatestParameters = new AimingParameters(
				targetVehicleDirection,
				Rotation2d.fromDegrees(177.0 - 0.8 + mShotCompensationDegrees.getDouble(0.0)),
				//Rotation2d.fromDegrees(armAngleMapSingle.get(targetDistance) + mShotCompensationDegrees.getDouble(0.0)),
				feedVelocity);

		Logger.recordOutput("RobotState/AimingParameters/TargetDirection", mLatestParameters.driveHeading());
		Logger.recordOutput("RobotState/AimingParameters/TargetDistance", targetDistance);
		Logger.recordOutput("RobotState/AimingParameters/ArmAngle", mLatestParameters.armAngle());
		Logger.recordOutput(
				"RobotState/AimingParameters/DriveFeedVelocityRadPerS",
				mLatestParameters.driveFeedVelocity());
		Logger.recordOutput(
				"RobotState/AimingParameters/ShotCompensation",
				mShotCompensationDegrees.getDouble(0.0));

		return mLatestParameters;
	}

	@AutoLogOutput(key = "RobotState/FieldVelocity")
	public Twist2d getFieldVelocity() {
		Translation2d linearFieldVelocity = new Translation2d(mRobotVelocity.dx, mRobotVelocity.dy)
				.rotateBy(getEstimatedPose().getRotation());
		return new Twist2d(
				linearFieldVelocity.getX(), linearFieldVelocity.getY(), mRobotVelocity.dtheta);
	}

	@AutoLogOutput(key = "RobotState/EstimatedPose")
	public Pose2d getEstimatedPose() {
		return mPoseEstimator.getEstimatedPosition();
	}

	/**
	 * Predicts what our pose will be in the future. Allows separate translation and
	 * rotation
	 * lookaheads to account for varying latencies in the different measurements.
	 *
	 * @param translationLookaheadS The lookahead time for the translation of the
	 *                              robot
	 * @param rotationLookaheadS    The lookahead time for the rotation of the robot
	 * @return The predicted pose.
	 */
	public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
		return getEstimatedPose()
				.exp(
						new Twist2d(
								mRobotVelocity.dx * translationLookaheadS,
								mRobotVelocity.dy * translationLookaheadS,
								mRobotVelocity.dtheta * rotationLookaheadS));
	}

	public void setPose(Pose2d pose) {
		mPoseEstimator.resetPosition(
				RobotContainer.mSwerve.getGyroYaw(),
				mLastModulePositions,
				pose);
	}

	private void clampPoseToField() {
		// if out of bounds, clamp to field
		double estimatedXPos = mPoseEstimator.getEstimatedPosition().getX();
		double estimatedYPos = mPoseEstimator.getEstimatedPosition().getY();
		if (estimatedYPos < 0.0
				|| estimatedYPos > 8.35
				|| estimatedXPos < 0.0
				|| estimatedXPos > Units.feetToMeters(52)) {
			double clampedYPosition = MathUtil.clamp(estimatedYPos, 0.0, 8.35);
			double clampedXPosition = MathUtil.clamp(estimatedXPos, 0.0, Units.feetToMeters(52.0));
			this.setPose(new Pose2d(clampedXPosition, clampedYPosition, getEstimatedPose().getRotation()));
		}
	}

	private void updateFieldWidget() {
		Pose2d robotPose = getEstimatedPose();
		mField2d.setRobotPose(robotPose);
	}

	public void addFieldPose(String name, Pose2d pose) {
		if (pose != null) {
			mField2d.getObject(name).setPose(pose);
		}
	}

	public void addFieldTrajectory(String name, Trajectory traj) {
		if (traj != null) {
			mField2d.getObject(name).setTrajectory(traj);
		}
	}
}

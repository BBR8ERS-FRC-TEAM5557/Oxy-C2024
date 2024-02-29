package frc.robot.subsystems.swerve.module;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.team6328.Alert;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.swerve.module.ModuleIO.ModuleIOInputs;
import frc.robot.util.Util;

public class Module {
	private final ModuleIO m_io;
	private final ModuleIOInputs m_inputs = new ModuleIOInputs();

	private final int moduleNumber;
	private final String moduleLabel;
	private double resetIteration = 0;
	private double lastAngle;
	private SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(kDrivekS, kDrivekV);

	private final Alert driveMotorDisconnected;
	private final Alert turnMotorDisconnected;

	private static final LoggedTunableNumber drivekP = new LoggedTunableNumber("Swerve/Module/DrivekP",
			kDrivekP);
	private static final LoggedTunableNumber drivekD = new LoggedTunableNumber("Swerve/Module/DrivekD",
			kDrivekD);
	private static final LoggedTunableNumber drivekS = new LoggedTunableNumber("Swerve/Module/DrivekS",
			kDrivekS);
	private static final LoggedTunableNumber drivekV = new LoggedTunableNumber("Swerve/Module/DrivekV",
			kDrivekV);
	private static final LoggedTunableNumber anglekP = new LoggedTunableNumber("Swerve/Module/AnglekP",
			kAnglekP);
	private static final LoggedTunableNumber anglekD = new LoggedTunableNumber("Swerve/Module/AnglekD",
			kAnglekD);

	/**
	 * Create a new swerve module.
	 *
	 * @param io           the hardware-abstracted swerve module object
	 * @param moduleNumber the module number (0-3)
	 */
	public Module(ModuleIO io, int moduleNumber) {
		this.m_io = io;
		this.moduleNumber = moduleNumber;

		lastAngle = getState().angle.getDegrees();
		boolean resetSuccesful = false;
		int resetIteration = 0;
		while (!resetSuccesful && resetIteration < 5) {
			resetSuccesful = resetToAbsolute();
			resetIteration++;
		}


		switch (moduleNumber) {
			case 0:
				moduleLabel = "FL";
				break;
			case 1:
				moduleLabel = "FR";
				break;
			case 2:
				moduleLabel = "BL";
				break;
			case 3:
				moduleLabel = "BR";
				break;
			default:
				moduleLabel = "";
				break;
		}

		driveMotorDisconnected = new Alert(
				"Drive", moduleLabel + " drive motor disconnected!", Alert.AlertType.WARNING);
		turnMotorDisconnected = new Alert(
				"Drive", moduleLabel + " turn motor disconnected!", Alert.AlertType.WARNING);

		ShuffleboardTab tab = Shuffleboard.getTab("Swerve");
		ShuffleboardLayout container = tab.getLayout(moduleLabel + " Module", BuiltInLayouts.kList)
				.withSize(2, 2)
				.withPosition(2 * this.moduleNumber, 3);
		container.addNumber(
				"Absolute",
				() -> Util.truncate(Units.radiansToDegrees(m_inputs.angleAbsolutePositionRad), 2));
		container.addNumber(
				"Integrated",
				() -> Util.truncate(Units.radiansToDegrees(m_inputs.angleInternalPositionRad) % 360.0, 2));
		container.addNumber("Velocity", () -> Util.truncate(m_inputs.driveVelocityMetersPerSec, 2));
		container.addNumber("Applied Volts", () -> Util.truncate(m_inputs.driveAppliedVolts, 2));
	}

	/**
	 * Update this swerve module's inputs and log them.
	 *
	 * <p>
	 * This method must be invoked by the drivetrain subsystem's periodic method.
	 */
	public void updateAndProcessInputs() {
		m_io.updateInputs(m_inputs);
		Logger.processInputs(kSubsystemName + "/" + moduleLabel + " Module", m_inputs);

		// Update ff and controllers
		LoggedTunableNumber.ifChanged(
				hashCode(),
				() -> m_driveFeedforward = new SimpleMotorFeedforward(drivekS.get(), drivekV.get(), 0),
				drivekS,
				drivekV);
		LoggedTunableNumber.ifChanged(
				hashCode(), () -> m_io.setDrivePID(drivekP.get(), 0, drivekD.get()), drivekP, drivekD);
		LoggedTunableNumber.ifChanged(
				hashCode(), () -> m_io.setAnglePID(anglekP.get(), 0, anglekD.get()), anglekP, anglekD);

		// Display alerts
		driveMotorDisconnected.set(!m_inputs.driveMotorConnected);
		turnMotorDisconnected.set(!m_inputs.angleMotorConnected);

		if (Math.abs(m_inputs.angleInternalVelocityRadPerSec) < kAbsoluteResetMaxOmega) {
			if (++resetIteration >= kAbsoluteResetIterations) {
				resetIteration = 0;
				resetToAbsolute();
			}
		} else {
			resetIteration = 0;
		}

	}

	public SwerveModuleState runSetpoint(
			SwerveModuleState state, boolean isOpenLoop, boolean forceAngle) {
		state = SwerveModuleState.optimize(
				state, Rotation2d.fromRadians(m_inputs.angleAbsolutePositionRad));

		if (isOpenLoop) {
			double driveVoltage = 12.0 * state.speedMetersPerSecond / kTheoreticalMaxSpeed; // FIX ME: 6.0 volts for sim
																							// stuff
			m_io.setDriveVoltage(driveVoltage);
		} else {
			m_io.setDriveVelocity(state.speedMetersPerSecond, m_driveFeedforward.calculate(state.speedMetersPerSecond));
		}

		// Unless the angle is forced (e.g., X-stance), don't rotate the module if speed
		// is less then
		// 1%. This prevents jittering if the controller isn't tuned perfectly. Perhaps
		// more
		// importantly, it allows for smooth repeated movement as the wheel direction
		// doesn't reset
		// during pauses (e.g., multi-segmented auto paths).
		double angle;
		if (!forceAngle && Math.abs(state.speedMetersPerSecond) <= (kTheoreticalMaxSpeed * 0.01)) {
			angle = lastAngle;
		} else {
			angle = state.angle.getRadians();
		}
		m_io.setAnglePosition(angle);
		lastAngle = angle;

		return state;
	}

	public void stop() {
		m_io.stop();
	}

	/**
	 * Set the drive motor to the specified voltage. This is only used for
	 * characterization via the
	 * FeedForwardCharacterization command. The module will be set to 0 degrees
	 * throughout the
	 * characterization; as a result, the wheels don't need to be clamped to hold
	 * them straight.
	 *
	 * @param voltage the specified voltage for the drive motor
	 */
	public void setVoltageForCharacterization(double voltage) {
		m_io.setAnglePosition(0.0);
		lastAngle = 0.0;
		m_io.setDriveVoltage(voltage);
	}

	/**
	 * Set the steer motor to the specified voltage. This is only used for systems
	 * check via the
	 * Drivetrain Systems Check command. The module will spin slowly in a circle.
	 *
	 * @param voltage the specified voltage for the drive motor
	 */
	public void setVoltageForAzimuthCheck(double voltage) {
		m_io.setAngleVoltage(voltage);
		lastAngle = m_inputs.angleAbsolutePositionRad;
		m_io.setDriveVoltage(0.0);
	}

	/**
	 * Get the current state of this swerve module.
	 *
	 * @return the current state of this swerve module
	 */
	public SwerveModuleState getState() {
		double velocity = m_inputs.driveVelocityMetersPerSec;
		Rotation2d angle = Rotation2d.fromRadians(m_inputs.angleAbsolutePositionRad);
		return new SwerveModuleState(velocity, angle);
	}

	/**
	 * Get the current position of this swerve module.
	 *
	 * @return the current position of this swerve module
	 */
	public SwerveModulePosition getPosition() {
		double distance = m_inputs.driveDistanceMeters;
		Rotation2d angle = Rotation2d.fromRadians(m_inputs.angleAbsolutePositionRad);
		return new SwerveModulePosition(distance, angle);
	}

	/**
	 * Get the number of this swerve module.
	 *
	 * @return the number of this swerve module
	 */
	public int getModuleNumber() {
		return moduleNumber;
	}

	/**
	 * Set the brake mode of the drive motor.
	 *
	 * @param enable if true, the drive motor will be set to brake mode; if false,
	 *               coast mode.
	 */
	public void setDriveBrakeMode(boolean enable) {
		m_io.setDriveBrakeMode(enable);
	}

	/**
	 * Set the brake mode of the angle motor.
	 *
	 * @param enable if true, the angle motor will be set to brake mode; if false,
	 *               coast mode.
	 */
	public void setAngleBrakeMode(boolean enable) {
		m_io.setAngleBrakeMode(enable);
	}

	public boolean resetToAbsolute() {
		return m_io.resetToAbsolute();
	}
}

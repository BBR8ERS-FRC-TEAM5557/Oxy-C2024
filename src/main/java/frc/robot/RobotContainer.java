// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.auto.SystemsCheckManager;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOKrakenSparkMax;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.util.DriveMotionPlanner;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FeedForwardCharacterization;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
	public static Swerve mSwerve;
	public static Intake mIntake;
	public static Feeder mFeeder;
	public static Flywheels mFlywheels;
	public static Arm mArm;

	public static final CommandXboxController mDriver = new CommandXboxController(0);
	public static final CommandXboxController mOperator = new CommandXboxController(1);
	private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).",
			AlertType.WARNING);
	private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 0).",
			AlertType.WARNING);

	public static RobotStateEstimator m_stateEstimator;
	public static SystemsCheckManager m_systemCheckManager;
	private final LoggedDashboardChooser<Command> mChooser;

	private PowerDistribution mPowerDistribution = new PowerDistribution();

	public RobotContainer() {
		if (kIsReal) {
			mSwerve = new Swerve(new GyroIOPigeon2(),
					new ModuleIOKrakenSparkMax(0, kFLDriveMotor, kFLTurnMotor, kFLOffset),
					new ModuleIOKrakenSparkMax(1, kFRDriveMotor, kFRTurnMotor, kFROffset),
					new ModuleIOKrakenSparkMax(2, kBLDriveMotor, kBLTurnMotor, kBLOffset),
					new ModuleIOKrakenSparkMax(3, kBRDriveMotor, kBRTurnMotor, kBROffset));
			mIntake = new Intake(new IntakeIOSparkMax());
			mFeeder = new Feeder(new FeederIOSparkMax());
			mFlywheels = new Flywheels(new FlywheelsIOKraken());
			mArm = new Arm(new ArmIOSparkMax());
		} else {
			mSwerve = new Swerve(new GyroIO() {
			}, new ModuleIOSim(), new ModuleIOSim(),
					new ModuleIOSim(), new ModuleIOSim());
		}

		// Instantiate missing subsystems
		if (mSwerve == null) {
			mSwerve = new Swerve(new GyroIO() {
			}, new ModuleIO() {
			}, new ModuleIO() {
			},
					new ModuleIO() {
					}, new ModuleIO() {
					});
		}
		if (mIntake == null) {
			mIntake = new Intake(new IntakeIO() {
			});
		}
		if (mFeeder == null) {
			mFeeder = new Feeder(new FeederIO() {
			});
		}
		if (mFlywheels == null) {
			mFlywheels = new Flywheels(new FlywheelsIO() {
			});
		}
		if (mArm == null) {
			mArm = new Arm(new ArmIO() {
			});
		}

		mChooser = new LoggedDashboardChooser<Command>("Driver/AutonomousChooser");

		SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
		SmartDashboard.putData("PDP", mPowerDistribution);

		m_systemCheckManager = new SystemsCheckManager(mSwerve);
		m_stateEstimator = RobotStateEstimator.getInstance();
		DriveMotionPlanner.configureControllers();

		configureBindings();
		generateEventMap();
		generateAutoChoices();
	}

	private void configureBindings() {
		// Bind driver and operator controls
		System.out.println("[Init] Binding controls");
		DriverStation.silenceJoystickConnectionWarning(true);

		/* SWERVING */
		TeleopDrive teleop = new TeleopDrive(this::getForwardInput, this::getStrafeInput,
				this::getRotationInput, this::getAimBotXInput, this::getAimBotYInput,
				this::getWantsAutoAimInput,
				this::getWantsSnapInput);
		mSwerve.setDefaultCommand(teleop.withName("Teleop Drive"));

		mDriver.start()
				.onTrue(Commands.runOnce(
						() -> m_stateEstimator.setPose(
								new Pose2d(
										m_stateEstimator.getEstimatedPose()
												.getTranslation(),
										AllianceFlipUtil.apply(
												new Rotation2d()))))
						.ignoringDisable(true).withName("ResetHeading"));
		mDriver.back()
				.onTrue(Commands.runOnce(
						() -> mSwerve.resetModuleEncoders())
						.ignoringDisable(true).withName("ResetSwerveEncoders"));

		// POTENTIALLY ADD IDK
		// mDriver.x().whileTrue(Commands.run(() -> mSwerve.stopWithX(), mSwerve));

		/* INTAKING */
		mOperator.rightBumper().whileTrue(
				mArm.intake().alongWith(Commands.waitUntil(mArm::atGoal)
						.andThen(Commands.parallel(mIntake.intake(), mFeeder.intake())
								.until(mFeeder::hasGamepiece)))
						.withName("Teleop Intaking"));

		mOperator.leftBumper().whileTrue(
				mArm.intake().alongWith(Commands.waitUntil(mArm::atGoal)
						.andThen(Commands.parallel(mIntake.eject(), mFeeder.ejectFloor())))
						.withName("Teleop Ejecting"));

		/* COASTING */
		mOperator.leftTrigger().whileTrue(Commands.parallel(mArm.forceCoast(), mSwerve.forceCoast())
				.ignoringDisable(true).withName("ForceCoast"));

		/* SHOOTING */
		mOperator.a().whileTrue(
				Commands.parallel(mArm.aimCustom(), mFlywheels.shoot()).withName("PrepCustomShot"));
		Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.a());
		mOperator.rightTrigger().and(mOperator.a())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(0.5),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("ScoreCustom"));

		mOperator.b()
				.whileTrue(Commands.parallel(mArm.aimFender(), mFlywheels.shootFender())
						.withName("PrepFenderShot"));
		Trigger readyToShootFender = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.b());
		mOperator.rightTrigger().and(mOperator.b())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(0.5),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("ScoreFender"));

		/* TRAPPING */
		mOperator.y().whileTrue(
				Commands.parallel(mArm.trap(), mFlywheels.prepareTrap().alongWith(mFeeder.prepareTrap())
						.raceWith((Commands.waitSeconds(1.0))).andThen(mFeeder.shootTrap()))
						.withName("PrepTrap"));
		Trigger readyToShootTrap = new Trigger(() -> mArm.atGoal()).and(mOperator.y());
		mOperator.rightTrigger().and(mOperator.y())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(0.5),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(Commands.parallel(mFlywheels.shootTrap(),
								mFeeder.shootTrap()))
						.withName("ScoreTrap"));

		/* CLIMBING */
		mOperator.pov(0).onTrue(mArm.prepClimb().alongWith(mFlywheels.stop()));
		mOperator.pov(180).onTrue(mArm.retractClimb().alongWith(mFlywheels.stop()));

		/* AMPING */
		mOperator.x().whileTrue(Commands.parallel(mArm.amp(), mFlywheels.eject()).withName("PrepAmp"));
		Trigger readyToEjectAmp = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.x());
		mOperator.rightTrigger().and(mOperator.x())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(0.5),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.ejectAmp()).withName("ScoreAmp"));

		/* SIGNALING */
		readyToShoot.or(readyToEjectAmp).or(readyToShootFender).or(readyToShootTrap)
				.whileTrue(
						Commands.run(
								() -> mOperator.getHID().setRumble(
										GenericHID.RumbleType.kBothRumble,
										1.0)))
				.whileFalse(
						Commands.run(
								() -> mOperator.getHID().setRumble(
										GenericHID.RumbleType.kBothRumble,
										0.0)));

		Command pulseControllers = Commands.sequence(Commands.runOnce(() -> {
			mDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
			mOperator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
		}).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
			mDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
			mOperator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
		}).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
			mDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
			mOperator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
		}).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
			mDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
			mOperator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
		})).withName("PulseControllers");

		Trigger pieceStaged = new Trigger(mFeeder::hasGamepiece);
		pieceStaged.onTrue(pulseControllers);

	}

	/** Updates the alerts for disconnected controllers. */
	public void checkControllers() {
		driverDisconnected.set(
				!DriverStation.isJoystickConnected(mDriver.getHID().getPort())
						|| !DriverStation.getJoystickIsXbox(mDriver.getHID().getPort()));
		operatorDisconnected.set(
				!DriverStation.isJoystickConnected(mOperator.getHID().getPort())
						|| !DriverStation.getJoystickIsXbox(mOperator.getHID().getPort()));
	}

	private void generateAutoChoices() {
		System.out.println("[Init] Auto Routines");

		mChooser.addDefaultOption("Do Nothing", null);
		mChooser.addDefaultOption("N3_S_C01", AutoBuilder.buildAuto("N3_S_C01"));
		mChooser.addOption("N4_S012_fender", AutoBuilder.buildAuto("N4_S012_fender"));
		mChooser.addOption("N4_S012", AutoBuilder.buildAuto("N4_S012"));
		mChooser.addDefaultOption("N5_S012_C4", AutoBuilder.buildAuto("N5_S012_C4"));
		mChooser.addDefaultOption("N6_S012_C34", AutoBuilder.buildAuto("N6_S012_C34"));

		// Set up feedforward characterization
		mChooser.addOption(
				"Drive FF Characterization",
				new FeedForwardCharacterization(
						mSwerve, mSwerve::runCharacterizationVolts,
						mSwerve::getCharacterizationVelocity)
						.finallyDo(mSwerve::stop));

		mChooser.addOption(
				"Flywheels FF Characterization",
				new FeedForwardCharacterization(
						mFlywheels, mFlywheels::runCharacterizationVolts,
						mFlywheels::getCharacterizationVelocity)
						.finallyDo(mFlywheels::stop));
	}

	private void generateEventMap() {
		NamedCommands.registerCommand("intakeNote",
				Commands.print("intaking started")
						.alongWith(mArm.intake()
								.alongWith(Commands.waitUntil(mArm::atGoal)
										.andThen(Commands.parallel(
												mIntake.intake(),
												mFeeder.intake())
												.until(mFeeder::hasGamepiece)))));

		Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal());

		NamedCommands.registerCommand("shootFender",
				Commands.print("shooting fender started")
						.alongWith(Commands.parallel(mArm.aimFender(), mFlywheels.shoot())
								.raceWith(Commands.waitUntil(readyToShoot)
										.andThen(mFeeder.shoot().alongWith(
												Commands.print("feeding started"))
												.until(() -> !mFeeder
														.hasGamepiece())))));

		NamedCommands.registerCommand("shootDistance",
				Commands.print("shooting distance started")
						.alongWith(Commands.parallel(mArm.aim(), mFlywheels.shoot())
								.raceWith(Commands.waitUntil(readyToShoot)
										.andThen(mFeeder.shoot().alongWith(
												Commands.print("feeding started"))
												.until(() -> !mFeeder
														.hasGamepiece())))));
	}

	public Command getAutonomousCommand() {
		return mChooser.get();
	}

	public Command getSubsystemCheckCommand() {
		return m_systemCheckManager.getCheckCommand();
	}

	public double getForwardInput() {
		return -square(deadband(mDriver.getLeftY(), 0.15));
	}

	public double getStrafeInput() {
		return -square(deadband(mDriver.getLeftX(), 0.15));
	}

	public double getRotationInput() {
		double leftTrigger = square(deadband(mDriver.getLeftTriggerAxis(), 0.05));
		double rightTrigger = square(deadband(mDriver.getRightTriggerAxis(), 0.05));

		return leftTrigger > rightTrigger ? leftTrigger : -rightTrigger;
	}

	public double getAimBotXInput() {
		return -(mDriver.getRightX());
	}

	public double getAimBotYInput() {
		return -(mDriver.getRightY());
	}

	public boolean getWantsAutoAimInput() {
		return mDriver.a().getAsBoolean();
	}

	public boolean getWantsSnapInput() {
		return mDriver.rightBumper().getAsBoolean();
	}

	public double getArmJogger() {
		return -square(deadband(mOperator.getRightY(), 0.15));
	}

	private static double deadband(double value, double tolerance) {
		if (Math.abs(value) < tolerance)
			return 0.0;
		return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
	}

	public static double square(double value) {
		return Math.copySign(value * value, value);
	}

	public static double cube(double value) {
		return Math.copySign(value * value * value, value);
	}
}
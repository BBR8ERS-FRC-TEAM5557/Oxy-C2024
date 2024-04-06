// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.auto.SystemsCheckManager;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOSparkMax;
import frc.robot.subsystems.blower.Blower;
import frc.robot.subsystems.blower.BlowerIO;
import frc.robot.subsystems.blower.BlowerIOSparkMax;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOKrakenSparkMax;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.subsystems.swerve.util.DriveMotionPlanner;
import frc.robot.subsystems.vision.AprilTagVisionIO;
import frc.robot.subsystems.vision.AprilTagVisionIOPhotonvision;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.ClimbLocation;
import frc.robot.util.FeedForwardCharacterization;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.cameraPoses;
import static frc.robot.subsystems.vision.AprilTagVisionConstants.instanceNames;

import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class RobotContainer {
	public static final CommandXboxController mDriver = new CommandXboxController(0);
	public static final CommandXboxController mOperator = new CommandXboxController(1);

	public static Swerve mSwerve;
	public static Intake mIntake;
	public static Feeder mFeeder;
	public static Flywheels mFlywheels;
	public static Arm mArm;
	public static Blower mBlower;
	public static Leds mLeds;

	public static Vision mVision;
	public static RobotStateEstimator mStateEstimator;
	public static SystemsCheckManager mSystemCheckManager;

	private final LoggedDashboardChooser<Command> mChooser;
	public static LoggedDashboardChooser<ClimbLocation> mClimbChooser;
	public static LoggedDashboardBoolean mVisionEnabled;

	private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).",
			AlertType.WARNING);
	private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
			AlertType.WARNING);

	public RobotContainer() {
		mLeds = Leds.getInstance();
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
			// mBlower = new Blower(new BlowerIOSparkMax());

			mVision = new Vision(
					new AprilTagVisionIOPhotonvision(instanceNames[0],
							GeometryUtil.pose3dToTransform3d(cameraPoses[0])),
					new AprilTagVisionIOPhotonvision(instanceNames[1],
							GeometryUtil.pose3dToTransform3d(cameraPoses[1])));

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
		if (mBlower == null) {
			mBlower = new Blower(new BlowerIO() {
			});
		}
		if (mVision == null) {
			mVision = new Vision(
					new AprilTagVisionIO() {
					},
					new AprilTagVisionIO() {
					});
		}

		mChooser = new LoggedDashboardChooser<Command>("Driver/AutonomousChooser");

		mClimbChooser = new LoggedDashboardChooser<ClimbLocation>("Driver/ClimbChooser");
		mClimbChooser.addDefaultOption("Left", ClimbLocation.STAGE_LEFT);
		mClimbChooser.addOption("Center", ClimbLocation.STAGE_CENTER);
		mClimbChooser.addOption("Right", ClimbLocation.STAGE_RIGHT);

		mVisionEnabled = new LoggedDashboardBoolean("Driver/VisionEnabled", true);

		if (Constants.kTuningMode) {
			SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
			// SmartDashboard.putData("PDH", mPowerDistribution);
		}

		// Alerts for constants
		if (Constants.kTuningMode) {
			new Alert("Tuning mode enabled, expect slower network", AlertType.INFO).set(true);
		}
		if (BurnManager.shouldBurn()) {
			new Alert("Burning flash enabled, consider disabling before competeing", AlertType.INFO).set(true);
		}

		mSystemCheckManager = new SystemsCheckManager(mSwerve);
		mStateEstimator = RobotStateEstimator.getInstance();
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
				this::getWantsCustomSnapInput,
				this::getWantsAmpSnapInput,
				this::getWantsClimbSnapInput,
				this::getWantsSnapInput);
		mSwerve.setDefaultCommand(teleop.withName("Teleop Drive"));

		mDriver.start()
				.onTrue(Commands.runOnce(
						() -> mStateEstimator.setPose(
								new Pose2d(
										mStateEstimator.getEstimatedPose()
												.getTranslation(),
										AllianceFlipUtil.apply(
												new Rotation2d()))))
						.ignoringDisable(true).withName("ResetHeading"));
		mDriver.back()
				.onTrue(Commands.runOnce(
						() -> mSwerve.resetModuleEncoders())
						.ignoringDisable(true).withName("ResetSwerveEncoders"));

		/* INTAKING */
		mOperator.rightBumper().whileTrue(
				mArm.intake().alongWith(Commands.waitUntil(mArm::atGoalRough)
						.andThen(Commands.parallel(mIntake.intake(), mFeeder.intake())
								.until(mFeeder::hasGamepiece)))
						.withName("Teleop Intaking"));

		mOperator.leftBumper()
				.whileTrue(Commands.parallel(mIntake.eject(), mFeeder.ejectFloor())
						.withName("Teleop Ejecting"));

		/* COASTING */
		mOperator.leftTrigger().and(() -> DriverStation.isDisabled())
				.whileTrue(Commands
						.parallel(mArm.forceCoast(), mSwerve.forceCoast(),
								new StartEndCommand(() -> Leds.getInstance().armCoast = true,
										() -> Leds.getInstance().armCoast = false))
						.ignoringDisable(true).withName("ForceCoast"));

		/* SHOOTING */
		mOperator.back().whileTrue(mFlywheels.shoot().withName("SpinUpFlywheels"));
		mOperator.start().onTrue(Commands.runOnce(() -> mVisionEnabled.set(!mVisionEnabled.get())));

		mOperator.pov(90)
				.onTrue(Commands.runOnce(() -> mStateEstimator.adjustShotCompensation(0.05)).ignoringDisable(true));
		mOperator.pov(270)
				.onTrue(Commands.runOnce(() -> mStateEstimator.adjustShotCompensation(-0.05)).ignoringDisable(true));

		mOperator.a().whileTrue(
				Commands.parallel(mArm.aim(), mFlywheels.shootDynamic()).withName("PrepDynamicShot"));
		Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.a());
		mOperator.rightTrigger().and(mOperator.a())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(1.0),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("ScoreCustom"));

		mOperator.leftTrigger().whileTrue(
				Commands.parallel(mArm.aimCustom(), mFlywheels.shoot()).withName("PrepDynamicShot"));
		Trigger readyToShootCustom = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.leftTrigger());
		mOperator.rightTrigger().and(mOperator.leftTrigger())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(1.0),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("ScoreCustom"));

		mOperator.b()
				.whileTrue(Commands.parallel(mArm.aimFender(), mFlywheels.shootFender(), mBlower.blow())
						.withName("PrepFenderShot"));
		Trigger readyToShootFender = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.b());
		mOperator.rightTrigger().and(mOperator.b())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(1.0),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("ScoreFender"));

		/* TRAPPING */
		Trigger trapTrigger = new Trigger(mOperator.leftTrigger().and(() -> !DriverStation.isDisabled()));
		/*
		 * trapTrigger.whileTrue(
		 * Commands.parallel(mArm.trap(), mBlower.blow(),
		 * mFlywheels.prepareTrap().alongWith(mFeeder.prepareTrap())
		 * .raceWith((Commands.waitSeconds(2.0))).andThen(mFeeder.shootTrap()))
		 * .withName("PrepTrap"));
		 */
		Trigger readyToShootTrap = new Trigger(() -> mArm.atGoal()).and(trapTrigger);
		/*
		 * mOperator.rightTrigger().and(trapTrigger)
		 * .onTrue(Commands.parallel(
		 * Commands.waitSeconds(2.5),
		 * Commands.waitUntil(mOperator.rightTrigger().negate()))
		 * .deadlineWith(Commands.parallel(mFlywheels.shootTrap(),
		 * mFeeder.shootTrap(), mBlower.blow()))
		 * .finallyDo(
		 * () -> new RunCommand(() -> mFlywheels.stop().schedule()).withTimeout(15.0)
		 * .schedule())
		 * .withName("ScoreTrap"));
		 */

		/* PASSING */
		mOperator.y()
				.whileTrue(Commands.parallel(mArm.pass(), mFlywheels.pass())
						.withName("PrepPassShot"));
		Trigger readyToPass = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.y());
		mOperator.rightTrigger().and(mOperator.y())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(0.5),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mFeeder.shoot()).withName("PassNote"));

		/* CLIMBING */
		mOperator.pov(0).onTrue(mArm.prepClimb().alongWith(mFlywheels.stop()));
		mOperator.pov(180).onTrue(mArm.retractClimb().alongWith(mFlywheels.stop()));

		/* AMPING */
		mOperator.x().whileTrue(Commands.parallel(mArm.prepAmp(), mFlywheels.eject()).withName("PrepAmp"));
		Trigger readyToEjectAmp = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.x());
		mOperator.rightTrigger().and(mOperator.x())
				.onTrue(Commands.parallel(
						Commands.waitSeconds(1.0),
						Commands.waitUntil(mOperator.rightTrigger().negate()))
						.deadlineWith(mArm.amp()
								.alongWith(Commands.waitUntil(() -> mArm.atGoalRough()).andThen(mFeeder.ejectAmp())))
						.withName("ScoreAmp"));

		PathPlannerPath path = PathPlannerPath.fromPathFile("alignAmp");
		PathConstraints constraints = new PathConstraints(
				4.0, 4.5,
				Units.degreesToRadians(540), Units.degreesToRadians(720));
		Command pathfindToAmp = AutoBuilder.pathfindThenFollowPath(
				path,
				constraints,
				0.0);
		//mDriver.rightBumper().and(mDriver.x()).whileTrue(pathfindToAmp);

		/* SIGNALING */
		Trigger inWing = new Trigger(
				() -> AllianceFlipUtil.apply(mStateEstimator.getEstimatedPose().getX()) < FieldConstants.wingX);
		inWing.whileTrue(
				new StartEndCommand(() -> Leds.getInstance().inWing = true, () -> Leds.getInstance().inWing = false));

		readyToShoot.or(readyToEjectAmp).or(readyToShootFender).or(readyToPass).or(readyToShootTrap).or(readyToShootCustom)
				.whileTrue(
						Commands.startEnd(
								() -> {
									mOperator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
									mLeds.readyForAction = true;
								},
								() -> {
									mOperator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
									mLeds.readyForAction = false;
								}));

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
		})).withName("PulseControllers").ignoringDisable(true);

		Trigger pieceStaged = new Trigger(mFeeder::hasGamepiece).debounce(0.25);
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
		mChooser.addDefaultOption("Drive Back", AutoBuilder.buildAuto("DriveBack"));

		mChooser.addDefaultOption("N3_S_C01", AutoBuilder.buildAuto("N3_S_C01"));

		// mChooser.addOption("N4_S012_fender",
		// AutoBuilder.buildAuto("N4_S012_fender"));
		mChooser.addOption("N4_C012", AutoBuilder.buildAuto("N4_C012"));
		mChooser.addOption("N4_S012", AutoBuilder.buildAuto("N4_S012"));
		mChooser.addOption("N4_S210", AutoBuilder.buildAuto("N4_S210"));

		mChooser.addDefaultOption("N5_S012_C4", AutoBuilder.buildAuto("N5_S012_C4"));

		mChooser.addDefaultOption("N6_S012_C43", AutoBuilder.buildAuto("N6_S012_C43"));

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
		Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal());
		Trigger headingReady = new Trigger(() -> mSwerve.atHeadingGoal());

		Trigger inWing = new Trigger(
				() -> AllianceFlipUtil.apply(mStateEstimator.getEstimatedPose().getX()) < FieldConstants.wingX);

		NamedCommands.registerCommand("intakeNote",
				Commands.print("intaking started")
						.alongWith(Commands.runOnce(() -> RobotStateEstimator.getInstance().getAimingParameters()))
						.alongWith(mArm.intake()
								.alongWith(Commands.waitUntil(mArm::atGoalRough)
										.andThen(Commands.parallel(
												mIntake.intake(),
												mFeeder.intake())))
								.until(mFeeder::hasGamepiece)));

		NamedCommands.registerCommand("trackGoal",
				Commands.print("tracking goal"));// .alongWith(Commands.sequence(Commands.waitUntil(inWing),
													// mArm.aim())));

		NamedCommands.registerCommand("shootFender",
				Commands.print("shooting fender started")
						.alongWith(Commands.parallel(mArm.aimFender(), mFlywheels.shootFender())
								.raceWith(Commands.waitUntil(() -> mFlywheels.atGoalFender())
										.andThen(mFeeder.shoot().alongWith(
												Commands.print("feeding started"))))
								.until(() -> !mFeeder
										.hasGamepiece())));

		NamedCommands.registerCommand("shootDistance",
				Commands.print("shooting distance started").alongWith(Commands.runOnce(() -> {
					RobotStateEstimator.getInstance().getAimingParameters();
					mVisionEnabled.set(true);
				}))
						.alongWith(Commands
								.parallel(mArm.aim(), mFlywheels.shootDynamic(),
										new RunCommand(() -> mSwerve.snapToSpeaker()))
								.raceWith(Commands
										.parallel(Commands.waitUntil(readyToShoot).withTimeout(2.0),
												Commands.waitUntil(headingReady).withTimeout(1.5))
										.andThen(mFeeder.shoot().alongWith(
												Commands.print("feeding started"))))
								.until(() -> !mFeeder
										.hasGamepiece())));
						//.finallyDo(() -> mVisionEnabled.set(true)));

		NamedCommands.registerCommand("raiseShot", Commands.print("adjusting to shoot higher")
				.alongWith(new InstantCommand(() -> mStateEstimator.adjustShotCompensation(-0.75))));

		NamedCommands.registerCommand("lowerShot", Commands.print("adjusting to shoot lower")
				.alongWith(new InstantCommand(() -> mStateEstimator.adjustShotCompensation(1.0))));
	}

	public Command getAutonomousCommand() {
		return mChooser.get();
	}

	public Command getSubsystemCheckCommand() {
		return mSystemCheckManager.getCheckCommand();
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

		// return -square(deadband(mDriver.getRightX(), 0.05));

		return leftTrigger > rightTrigger ? leftTrigger : -rightTrigger;
	}

	public double getAimBotXInput() {
		return -(mDriver.getRightX());
	}

	public double getAimBotYInput() {
		return -(mDriver.getRightY());
	}

	public boolean getWantsAutoAimInput() {
		return mDriver.getHID().getLeftBumper();
	}

	public boolean getWantsCustomSnapInput() {
		return mDriver.getHID().getAButton();
	}

	public boolean getWantsAmpSnapInput() {
		return mDriver.getHID().getXButton();
	}

	public boolean getWantsClimbSnapInput() {
		return mDriver.getHID().getYButton();
	}

	public boolean getWantsSnapInput() {
		return mDriver.getHID().getRightBumper();
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
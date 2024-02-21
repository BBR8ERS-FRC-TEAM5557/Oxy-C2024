// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoRoutineManager;
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
import frc.robot.util.FieldConstants;
import frc.robot.util.RobotStateEstimator;
import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;

import java.util.HashMap;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {

    public static final CommandXboxController mDriver = new CommandXboxController(0);
    public static final CommandXboxController mOperator = new CommandXboxController(1);

    private final LoggedDashboardChooser<Command> mChooser;

    public static Swerve mSwerve;
    public static Intake mIntake;
    public static Feeder mFeeder;
    public static Flywheels mFlywheels;
    public static Arm mArm;

    public static RobotStateEstimator m_stateEstimator;
    public static SystemsCheckManager m_systemCheckManager;

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
                this::getRotationInput, this::getAimBotXInput, this::getAimBotYInput, this::getAutoAimInput);
        mSwerve.setDefaultCommand(teleop);

        mDriver.start()
                .onTrue(Commands.runOnce(
                        () -> m_stateEstimator.setPose(
                                new Pose2d(
                                        m_stateEstimator.getPose().getTranslation(),
                                        AllianceFlipUtil.apply(new Rotation2d()))))
                        .ignoringDisable(true));
        mDriver.back()
                .onTrue(Commands.runOnce(
                        () -> m_stateEstimator.setPose(
                                AllianceFlipUtil.apply(
                                        new Pose2d(
                                                Units.inchesToMeters(36.0),
                                                FieldConstants.Speaker.centerSpeakerOpening.getY(),
                                                new Rotation2d()))))
                        .ignoringDisable(true));

        mDriver.x().onTrue(Commands.runOnce(() -> mSwerve.stopWithX()));

        /* INTAKING */
        mOperator.rightBumper()
                .whileTrue(mArm.intake()
                        .alongWith(Commands.waitUntil(mArm::atGoal)
                                .andThen(Commands.parallel(mIntake.intake(), mFeeder.intake())
                                        .until(() -> mFeeder.hasGamepiece()))));

        mOperator.leftBumper()
                .whileTrue(mArm.intake()
                        .alongWith(Commands.waitUntil(mArm::atGoal)
                                .andThen(Commands.parallel(mIntake.eject(), mFeeder.ejectFloor()))));

        /* COASTING */
        mOperator.leftBumper().whileTrue(mArm.forceCoast());

        /* SHOOTING */
        mOperator.a().whileTrue(Commands.parallel(mArm.aimCustom(), mFlywheels.shoot()));
        Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.a());
        mOperator.rightTrigger().and(mOperator.a())
                .onTrue(Commands.parallel(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(mOperator.rightTrigger().negate()))
                        .deadlineWith(mFeeder.shoot()));

        mOperator.b().whileTrue(Commands.parallel(mArm.aimFender(), mFlywheels.shoot()));
        Trigger readyToShootFender = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.b());
        mOperator.rightTrigger().and(mOperator.b())
                .onTrue(Commands.parallel(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(mOperator.rightTrigger().negate()))
                        .deadlineWith(mFeeder.shoot()));

        mOperator.y().whileTrue(Commands.parallel(mArm.aimCustom(), mFlywheels.shoot()));
        Trigger readyToShootCustom = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.y());
        mOperator.rightTrigger().and(mOperator.y())
                .onTrue(Commands.parallel(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(mOperator.rightTrigger().negate()))
                        .deadlineWith(mFeeder.shoot()));

        /* AMPING */
        mOperator.x().whileTrue(Commands.parallel(mArm.amp(), mFlywheels.eject()));
        Trigger readyToEjectAmp = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal()).and(mOperator.x());
        mOperator.rightTrigger().and(mOperator.x())
                .onTrue(Commands.parallel(
                        Commands.waitSeconds(0.5),
                        Commands.waitUntil(mOperator.rightTrigger().negate()))
                        .deadlineWith(mFeeder.ejectAmp()));

        readyToShoot.or(readyToEjectAmp).or(readyToShootFender).or(readyToShootCustom)
                .whileTrue(
                        Commands.run(
                                () -> mOperator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0)))
                .whileFalse(
                        Commands.run(
                                () -> mOperator.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0)));
        /* CLIMBING */
        mOperator.pov(0).onTrue(mArm.prepClimb().alongWith(mFlywheels.stop()));
        mOperator.pov(180).onTrue(mArm.retractClimb());

    }

    private void generateAutoChoices() {
        System.out.println("[Init] Auto Routines");

        mChooser.addDefaultOption("Do Nothing", null);
        mChooser.addOption("fender-4Piece", AutoBuilder.buildAuto("fender-4Piece"));

        // Set up feedforward characterization
        mChooser.addOption(
                "Drive FF Characterization",
                new FeedForwardCharacterization(
                        mSwerve, mSwerve::runCharacterizationVolts, mSwerve::getCharacterizationVelocity)
                        .finallyDo(mSwerve::stop));

        mChooser.addOption(
                "Flywheels FF Characterization",
                new FeedForwardCharacterization(
                        mFlywheels, mFlywheels::runCharacterizationVolts, mFlywheels::getCharacterizationVelocity)
                        .finallyDo(mFlywheels::stop));
    }

    private void generateEventMap() {
        NamedCommands.registerCommand("intakeNote",
                Commands.print("Intaking started").andThen(mArm.intake()
                        .alongWith(Commands.waitUntil(mArm::atGoal)
                                .andThen(Commands.parallel(mIntake.intake(), mFeeder.intake())
                                        .raceWith(Commands.waitUntil(mFeeder::hasGamepiece))))));

        Trigger readyToShoot = new Trigger(() -> mArm.atGoal() && mFlywheels.atGoal());

        NamedCommands.registerCommand("shootFenderNote",
                Commands.print("shooting started")
                        .andThen(Commands.parallel(mArm.aimFender(), mFlywheels.shoot())
                                .raceWith(Commands.waitUntil(readyToShoot)
                                        .andThen(mFeeder.shoot().alongWith(Commands.print("feeding started"))
                                                .until(() -> !mFeeder.hasGamepiece())))));
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
        // return -square(deadband(m_driver.getRightX(), 0.15));
    }

    public double getAimBotXInput() {
        return -(mDriver.getRightX());
    }

    public double getAimBotYInput() {
        return -(mDriver.getRightY());
    }

    public boolean getAutoAimInput() {
        return mOperator.a().getAsBoolean();
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
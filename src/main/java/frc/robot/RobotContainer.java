// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoRoutineManager;
import frc.robot.auto.SystemsCheckManager;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.module.ModuleIOKrakenSparkMax;
import frc.robot.subsystems.swerve.module.ModuleIOSim;
import frc.robot.util.DriveMotionPlanner;
import frc.robot.util.RobotStateEstimator;
import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;

public class RobotContainer {

    public static final XboxController m_driver = new XboxController(0);
    public static final XboxController m_operator = new XboxController(1);
    public static Swerve m_swerve;
    public static Intake m_intake;

    public static RobotStateEstimator m_stateEstimator;

    public static AutoRoutineManager m_autoManager;
    public static SystemsCheckManager m_systemCheckManager;

    public RobotContainer() {
        if (kIsReal) {
            m_swerve = new Swerve(new GyroIOPigeon2(),
                    new ModuleIOKrakenSparkMax(0, kFLDriveMotor, kFLTurnMotor, kFLOffset),
                    new ModuleIOKrakenSparkMax(1, kFRDriveMotor, kFRTurnMotor, kFROffset),
                    new ModuleIOKrakenSparkMax(2, kBLDriveMotor, kBLTurnMotor, kBLOffset),
                    new ModuleIOKrakenSparkMax(3, kBRDriveMotor, kBRTurnMotor, kBROffset));
        } else {
            m_swerve = new Swerve(new GyroIO() {
            }, new ModuleIOSim(), new ModuleIOSim(),
                    new ModuleIOSim(), new ModuleIOSim());
        }

        // Instantiate missing subsystems
        if (m_swerve == null) {
            m_swerve = new Swerve(new GyroIO() {
            }, new ModuleIO() {
            }, new ModuleIO() {
            },
                    new ModuleIO() {
                    }, new ModuleIO() {
                    });
        }

        m_autoManager = new AutoRoutineManager(m_swerve);
        m_systemCheckManager = new SystemsCheckManager(m_swerve);
        m_stateEstimator = RobotStateEstimator.getInstance();
        DriveMotionPlanner.configureControllers();

        configureBindings();
    }

    private void configureBindings() {
        // Bind driver and operator controls
        System.out.println("[Init] Binding controls");
        DriverStation.silenceJoystickConnectionWarning(true);

        m_swerve.setDefaultCommand(new TeleopDrive(this::getForwardInput, this::getStrafeInput,
                this::getRotationInput, this::getAimBotXInput, this::getAimBotYInput));

        // Reset swerve heading
        new Trigger(m_driver::getStartButton)
                .onTrue(new InstantCommand(() -> m_stateEstimator.setPose(new Pose2d())));

    }

    public Command getAutonomousCommand() {
        return m_autoManager.getAutoCommand();
    }

    public Command getSubsystemCheckCommand() {
        return m_systemCheckManager.getCheckCommand();
    }

    public double getForwardInput() {
        return -square(deadband(m_driver.getLeftY(), 0.15));
    }

    public double getStrafeInput() {
        return -square(deadband(m_driver.getLeftX(), 0.15));
    }

    public double getRotationInput() {
        double leftTrigger = square(deadband(m_driver.getLeftTriggerAxis(), 0.05));
        double rightTrigger = square(deadband(m_driver.getRightTriggerAxis(), 0.05));

        return leftTrigger > rightTrigger ? leftTrigger : -rightTrigger;
        // return -square(deadband(m_driver.getRightX(), 0.15));
    }

    public double getAimBotXInput() {
        return -(m_driver.getRightX());
    }

    public double getAimBotYInput() {
        return -(m_driver.getRightY());
    }

    public double getArmJogger() {
        return -square(deadband(m_operator.getRightY(), 0.15));
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
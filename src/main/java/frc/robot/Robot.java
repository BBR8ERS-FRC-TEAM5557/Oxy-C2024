// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.subsystems.flywheels.Flywheels.IdleMode;
import frc.robot.subsystems.leds.Leds;
import frc.lib.team6328.VirtualSubsystem;

public class Robot extends LoggedRobot {
	private RobotContainer mRobotContainer;

	private Command mAutoCommand;
	private double autoStart;
	private boolean autoMessagePrinted;

	private Command mSubsystemCheckCommand;

	private final Timer canErrorTimer = new Timer();
	private final Timer disabledTimer = new Timer();

	private final Alert logReceiverQueueAlert = new Alert("Logging queue exceeded capacity, data will NOT be logged.",
			AlertType.ERROR);
	private final Alert canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.",
			AlertType.ERROR);
	private final Alert lowBatteryAlert = new Alert(
			"Battery voltage is very low, consider turning off the robot or replacing the battery.",
			AlertType.WARNING);

	@Override
	public void robotInit() {
		if (Constants.kIsReal) {
			Logger.addDataReceiver(new WPILOGWriter()); // gotta plug a usb stick into rio 2 to have logging
			Logger.addDataReceiver(new NT4Publisher());
			LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
		} else {
			Logger.addDataReceiver(new NT4Publisher());
		}
		Logger.start();

		// Log active commands
		Map<String, Integer> commandCounts = new HashMap<>();
		BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
			String name = command.getName();
			int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
			commandCounts.put(name, count);
			Logger.recordOutput(
					"CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
			Logger.recordOutput("CommandsAll/" + name, count > 0);
		};
		CommandScheduler.getInstance()
				.onCommandInitialize(
						(Command command) -> {
							logCommandFunction.accept(command, true);
						});
		CommandScheduler.getInstance()
				.onCommandFinish(
						(Command command) -> {
							logCommandFunction.accept(command, false);
						});
		CommandScheduler.getInstance()
				.onCommandInterrupt(
						(Command command) -> {
							logCommandFunction.accept(command, false);
						});

		// Default to blue alliance in sim
		if (!Constants.kIsReal) {
			DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
		}

		// Start timers
		canErrorTimer.reset();
		canErrorTimer.start();
		disabledTimer.reset();
		disabledTimer.start();

		// Instantiate RobotContainer
		System.out.println("[Init] Instantiating RobotContainer");
		mRobotContainer = new RobotContainer();
	}

	@Override
	public void robotPeriodic() {
		Threads.setCurrentThreadPriority(true, 99);
		CommandScheduler.getInstance().run();
		VirtualSubsystem.periodicAll();

		// Print auto duration
		if (mAutoCommand != null) {
			if (!mAutoCommand.isScheduled() && !autoMessagePrinted) {
				if (DriverStation.isAutonomousEnabled()) {
					System.out.printf(
							"*** Auto finished in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
				} else {
					System.out.printf(
							"*** Auto cancelled in %.2f secs ***%n", Timer.getFPGATimestamp() - autoStart);
				}
				autoMessagePrinted = true;
				Leds.getInstance().autoFinished = true;
				Leds.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
			}
		}

		// Check logging fault
		logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

		// Update CAN error alert
		var canStatus = RobotController.getCANStatus();
		if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
			canErrorTimer.reset();
		}
		canErrorAlert.set(!canErrorTimer.hasElapsed(0.5));

		// Update low battery alert
		if (DriverStation.isEnabled()) {
			disabledTimer.reset();
		}
		if (RobotController.getBatteryVoltage() < 11.5 && disabledTimer.hasElapsed(2.0)) {
			Leds.getInstance().lowBatteryAlert = true;
			lowBatteryAlert.set(true);
		}

		// Robot container periodic methods
		mRobotContainer.checkControllers();
		RobotStateEstimator.getInstance().getAimingParameters();

		Threads.setCurrentThreadPriority(true, 10);
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
	}

	@Override
	public void disabledExit() {
	}

	@Override
	public void autonomousInit() {
		autoStart = Timer.getFPGATimestamp();
		autoMessagePrinted = false;
		mAutoCommand = RobotContainer.mAutoChooser.get();
		RobotContainer.mVisionEnabled.set(true);
		
		mSubsystemCheckCommand = mRobotContainer.getSubsystemCheckCommand();

		if (mAutoCommand != null) {
			RobotContainer.mFlywheels.setIdleMode(IdleMode.AUTO);
			mAutoCommand.schedule();

		} else if (!DriverStation.isFMSAttached() && mSubsystemCheckCommand != null) {
			System.out.println("SystemCheck Started");
			mSubsystemCheckCommand.schedule();
		} else {
			System.out.println("No Auto Rountine Selected!!!");
		}
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void autonomousExit() {
		if (mSubsystemCheckCommand != null) {
			mSubsystemCheckCommand.cancel();
		}
	}

	@Override
	public void teleopInit() {
		RobotContainer.mVisionEnabled.set(true);
		if (mAutoCommand != null) {
			mAutoCommand.cancel();
		}
		RobotContainer.mFlywheels.setIdleMode(IdleMode.TELEOP);
	}

	@Override
	public void teleopPeriodic() {
	}

	@Override
	public void teleopExit() {
	}

	@Override
	public void testInit() {
		CommandScheduler.getInstance().cancelAll();
	}

	@Override
	public void testPeriodic() {
	}

	@Override
	public void testExit() {
	}
}

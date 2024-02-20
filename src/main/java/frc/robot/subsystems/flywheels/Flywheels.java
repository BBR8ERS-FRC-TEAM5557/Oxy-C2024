package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.*;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.flywheels.FlywheelsIO.FlywheelsIOInputs;
import frc.robot.util.Util;
import frc.lib.team6328.Alert;
import frc.lib.team6328.LoggedTunableNumber;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", kFlywheelP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", kFlywheelI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", kFlywheelI);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", kFlywheelS);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", kFlywheelV);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", kFlywheelA);

    private static final LoggedTunableNumber mShootingRpm = new LoggedTunableNumber("Flywheels/ShootingRpm", 7000.0);
    private static final LoggedTunableNumber mIdleRpm = new LoggedTunableNumber("Flywheels/IdleRpm", 1500.0);
    private static final LoggedTunableNumber mIntakingRpm = new LoggedTunableNumber("Flywheels/IntakingRpm", -2000.0);
    private static final LoggedTunableNumber mEjectingRpm = new LoggedTunableNumber("Flywheels/EjectingRpm", 1500.0);

    private final FlywheelsIO mIO;
    private final FlywheelsIOInputs mInputs = new FlywheelsIOInputs();

    private State mState = State.IDLE;

    private SimpleMotorFeedforward mFeedforward = new SimpleMotorFeedforward(kFlywheelS, kFlywheelV, kFlywheelA);

    private final Alert leftMotorDisconnected;
    private final Alert rightMotorDisconnected;

    @RequiredArgsConstructor
    public enum State {
        STOP(() -> 0),
        IDLE(mIdleRpm),
        SHOOT(mShootingRpm),
        INTAKE(mIntakingRpm),
        EJECT(mEjectingRpm),
        CHARACTERIZING(() -> 0.0);

        private final DoubleSupplier rpm;

        private double getRPM() {
            return rpm.getAsDouble();
        }
    }

    public Flywheels(FlywheelsIO io) {
        this.mIO = io;

        leftMotorDisconnected = new Alert(
                "Flywheels", "Left flywheel motor disconnected!", Alert.AlertType.WARNING);
        rightMotorDisconnected = new Alert(
                "Flywheels", "Right flywheel motor disconnected!", Alert.AlertType.WARNING);

        setDefaultCommand(runOnce(() -> setState(State.IDLE)).withName("Flywheels Idle"));
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Flywheels", mInputs);

        // check controllers
        LoggedTunableNumber.ifChanged(hashCode(), () -> mIO.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(hashCode(),
                () -> mFeedforward = new SimpleMotorFeedforward(kS.get(), kV.get(), kA.get()), kS, kV, kA);

        // Stop when disabled
        if (DriverStation.isDisabled()) {
            setState(State.STOP);
        }

        if(mState != State.CHARACTERIZING)
            mIO.runVelocity(mState.getRPM(), mFeedforward.calculate(mState.getRPM()));

        // Display alerts
        leftMotorDisconnected.set(!mInputs.leftMotorConnected);
        rightMotorDisconnected.set(!mInputs.rightMotorConnected);

        Logger.recordOutput("Flywheels/State", mState);
        Logger.recordOutput("Flywheels/SetpointRpm", mState.getRPM());
        Logger.recordOutput("Flywheels/AtGoal", atGoal());
    }

    private void setState(State state) {
        this.mState = state;
    }

    public void runCharacterizationVolts(double volts) {
        setState(State.CHARACTERIZING);
        mIO.runVoltage(volts);
    }

    public double getCharacterizationVelocity() {
        return (mInputs.leftVelocityRpm + mInputs.rightVelocityRpm) / 2.0;
    }

    public boolean atGoal() {
        double goalRpm = mState.getRPM();
        return Util.epsilonEquals(mInputs.leftVelocityRpm, goalRpm, kPadding)
                || Util.epsilonEquals(mInputs.rightVelocityRpm, goalRpm, kPadding);
    }

    public Command shoot() {
        return startEnd(() -> setState(State.SHOOT), () -> setState(State.IDLE)).withName("FlywheelsShoot");
    }

    public Command eject() {
        return startEnd(() -> setState(State.EJECT), () -> setState(State.IDLE))
                .withName("FlywheelsEject");
    }

    public Command intake() {
        return startEnd(() -> setState(State.INTAKE), () -> setState(State.IDLE))
                .withName("FlywheelsIntake");
    }

    public Command stop() {
        return runOnce(() -> setState(State.STOP));
    }
}
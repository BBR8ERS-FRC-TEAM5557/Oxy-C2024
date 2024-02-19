package frc.robot.subsystems.feeder;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import lombok.RequiredArgsConstructor;

public class Feeder extends SubsystemBase {
    private final FeederIO mIO;
    private final FeederIOInputs mInputs = new FeederIOInputs();

    private State mState = State.STOP;

    private static final LoggedTunableNumber mIntakeVoltage = new LoggedTunableNumber("Feeder/IntakeVoltage", 1.5);
    private static final LoggedTunableNumber mShootVoltage = new LoggedTunableNumber("Feeder/ShootingVoltage", 8.0);
    private static final LoggedTunableNumber mIdleVoltage = new LoggedTunableNumber("Feeder/IdleVoltage", 0.0);
    private static final LoggedTunableNumber mEjectingAmpVoltage = new LoggedTunableNumber("Feeder/EjectingAmpVoltage",
            4.0);
    private static final LoggedTunableNumber mEjectingFloorVoltage = new LoggedTunableNumber(
            "Feeder/EjectingFloorVoltage", -8.0);

    private static final LoggedTunableNumber mVelocityThreshold = new LoggedTunableNumber("Feeder/VelocityThreshold",
            200.0);
    private static final LoggedTunableNumber mCurrentThreshold = new LoggedTunableNumber("Feeder/CurrentThreshold",
            25.0);

    public Feeder(FeederIO io) {
        System.out.println("[Init] Creating Intake");
        this.mIO = io;
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Intake", mInputs);

        if (DriverStation.isDisabled()) {
            setState(State.IDLE);
        }

        mIO.setFeederVoltage(this.mState.getMotorVoltage());

        Logger.recordOutput("state", mState);
        Logger.recordOutput("isStalled", isStalled());
    }

    private void setState(State state) {
        this.mState = state;
    }

    public double getMotorCurrent() {
        return mInputs.feederCurrentAmps;
    }

    public double getMotorRPM() {
        return mInputs.feederVelocityRPM;
    }

    public double getMotorOutput() {
        return mInputs.feederAppliedVolts;
    }

    public boolean isStalled() {
        return Math.abs(mInputs.feederVelocityRPM) <= mVelocityThreshold.get();
    }

    public boolean hasGamepiece() {
        return mInputs.hasGamepiece;
    }

    @RequiredArgsConstructor
    public enum State {
        INTAKE(mIntakeVoltage),
        SHOOT(mShootVoltage),
        IDLE(mIdleVoltage),
        EJECT_AMP(mEjectingAmpVoltage),
        EJECT_FLOOR(mEjectingFloorVoltage),
        STOP(() -> 0.0);

        private final DoubleSupplier motorVoltage;

        public double getMotorVoltage() {
            return motorVoltage.getAsDouble();
        }
    }

    public Command intake() {
        return startEnd(() -> setState(State.INTAKE), () -> setState(State.IDLE)).withName("FeederPickup");
    }

    public Command shoot() {
        return startEnd(() -> setState(State.SHOOT), () -> setState(State.IDLE)).withName("FeederShoot");
    }

    public Command ejectAmp() {
        return startEnd(() -> setState(State.EJECT_AMP), () -> setState(State.IDLE)).withName("FeederEjectAmp");
    }

    public Command ejectFloor() {
        return startEnd(() -> setState(State.EJECT_FLOOR), () -> setState(State.IDLE)).withName("FeederEjectFloor");
    }

    public Command idle() {
        return runOnce(() -> setState(State.IDLE));
    }

}
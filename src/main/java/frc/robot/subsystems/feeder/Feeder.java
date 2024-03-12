package frc.robot.subsystems.feeder;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.feeder.FeederIO.FeederIOInputs;
import frc.robot.subsystems.leds.Leds;
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

    private static final LoggedTunableNumber mPreppingTrapVoltage = new LoggedTunableNumber(
            "Feeder/PreppingTrapVoltage", 1.0);
    private static final LoggedTunableNumber mShootTrapVoltage = new LoggedTunableNumber(
            "Feeder/ShootTrapVoltage", -12.0);


    public Feeder(FeederIO io) {
        System.out.println("[Init] Creating Intake");
        this.mIO = io;
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Feeder", mInputs);

        if (DriverStation.isDisabled()) {
            setState(State.IDLE);
        }

        mIO.setFeederVoltage(this.mState.getMotorVoltage());

        Leds.getInstance().hasNote = hasGamepiece();
        Leds.getInstance().intaking = mState == State.INTAKE;

        Logger.recordOutput("state", mState);
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

    public boolean hasGamepiece() {
        return mInputs.hasGamepiece;
    }

    @RequiredArgsConstructor
    public enum State {
        STOP(() -> 0.0),
        IDLE(mIdleVoltage),

        INTAKE(mIntakeVoltage),
        EJECT_FLOOR(mEjectingFloorVoltage),

        SHOOT(mShootVoltage),
        EJECT_AMP(mEjectingAmpVoltage),
        
        PREP_TRAP(mPreppingTrapVoltage),
        SHOOT_TRAP(mShootTrapVoltage);

        private final DoubleSupplier motorVoltage;

        public double getMotorVoltage() {
            return motorVoltage.getAsDouble();
        }
    }

    public Command intake() {
        return startEnd(() -> setState(State.INTAKE), () -> setState(State.IDLE)).withName("FeederIntake");
    }

    public Command shoot() {
        return startEnd(() -> setState(State.SHOOT), () -> setState(State.IDLE)).withName("FeederShoot");
    }

    public Command ejectAmp() {
        return startEnd(() -> setState(State.EJECT_AMP), () -> setState(State.IDLE)).withName("FeederEjectAmp");
    }

    public Command prepareTrap() {
        return startEnd(() -> setState(State.PREP_TRAP), () -> setState(State.STOP)).withName("FeederPrepTrap");
    }

    public Command shootTrap() {
        return startEnd(() -> setState(State.SHOOT_TRAP), () -> setState(State.IDLE)).withName("FeederShootTrap");
    }

    public Command ejectFloor() {
        return startEnd(() -> setState(State.EJECT_FLOOR), () -> setState(State.IDLE)).withName("FeederEjectFloor");
    }

    public Command idle() {
        return runOnce(() -> setState(State.IDLE)).withName("FeederIdle");
    }

}
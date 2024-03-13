package frc.robot.subsystems.intake;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;
import lombok.RequiredArgsConstructor;

public class Intake extends SubsystemBase {

    private final IntakeIO mIO;
    private final IntakeIOInputs mInputs = new IntakeIOInputs();

    private State mState = State.STOP;

    private static final LoggedTunableNumber mIntakeVoltage = new LoggedTunableNumber("Intake/IntakeVoltage", 12.0);
    private static final LoggedTunableNumber mIdleVoltage = new LoggedTunableNumber("Intake/IdleVoltage", 0.0);
    private static final LoggedTunableNumber mEjectingVoltage = new LoggedTunableNumber("Intake/EjectingVoltage", -8.0);

    private static final LoggedTunableNumber mVelocityThreshold = new LoggedTunableNumber("Intake/VelocityThreshold", 200.0);
    private static final LoggedTunableNumber mCurrentThreshold = new LoggedTunableNumber("Intake/CurrentThreshold", 25.0);

    public Intake(IntakeIO io) {
        System.out.println("[Init] Creating Intake");
        this.mIO = io;
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Intake", mInputs);

        if (DriverStation.isDisabled()) {
            setState(State.STOP);
        }
        
        mIO.setIntakeVoltage(this.mState.getTopMotorVoltage(), this.mState.getBottomMotorVoltage());

        Logger.recordOutput("state", mState);
        Logger.recordOutput("isStalled", isStalled());
    }

    private void setState(State state) {
        this.mState = state;
    }

    public double getMotorCurrent() {
        return mInputs.intakeTopCurrentAmps;
    }

    public double getMotorRPM() {
        return mInputs.intakeTopVelocityRPM;
    }

    public double getMotorOutput() {
        return mInputs.intakeTopAppliedVolts;
    }

    public boolean isStalled() {
        return Math.abs(mInputs.intakeTopVelocityRPM) <= mVelocityThreshold.get()
                && mInputs.intakeTopCurrentAmps >= mCurrentThreshold.get();
    }

    @RequiredArgsConstructor
    public enum State {
        INTAKE(mIntakeVoltage,mIntakeVoltage),
        IDLE(mIdleVoltage, mIdleVoltage),
        STOP(() -> 0.0, () -> 0.0),
        EJECT(mEjectingVoltage, mEjectingVoltage);

        private final DoubleSupplier topMotorVoltage;
        private final DoubleSupplier bottomMotorVoltage;

        public double getTopMotorVoltage() {
            return topMotorVoltage.getAsDouble();
        }

        public double getBottomMotorVoltage() {
            return bottomMotorVoltage.getAsDouble();
        }
    }

    public Command intake() {
        return startEnd(() -> setState(State.INTAKE), () -> setState(State.IDLE)).withName("IntakePickup");
    }

    public Command eject() {
        return startEnd(() -> setState(State.EJECT), () -> setState(State.IDLE)).withName("IntakeEject");
    }

    public Command idle() {
        return runOnce(() -> setState(State.IDLE));
    }
}
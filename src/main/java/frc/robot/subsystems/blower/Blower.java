package frc.robot.subsystems.blower;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.blower.BlowerIO.BlowerIOInputs;
import lombok.RequiredArgsConstructor;

public class Blower extends SubsystemBase {
    private final BlowerIO mIO;
    private final BlowerIOInputs mInputs = new BlowerIOInputs();

    private State mState = State.STOP;

    private static final LoggedTunableNumber mBlowVoltage = new LoggedTunableNumber("Blower/BlowVoltage", 12.0);

    public Blower(BlowerIO io) {
        System.out.println("[Init] Creating Blower");
        this.mIO = io;
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Blower", mInputs);

        if (DriverStation.isDisabled()) {
            setState(State.STOP);
        }

        mIO.setBlowerVoltage(this.mState.getBlowMotorVoltage());

        Logger.recordOutput("state", mState);
    }

    private void setState(State state) {
        this.mState = state;
    }

    @RequiredArgsConstructor
    public enum State {
        BLOW(mBlowVoltage),
        STOP(() -> 0.0);

        private final DoubleSupplier blowMotorVoltage;

        public double getBlowMotorVoltage() {
            return blowMotorVoltage.getAsDouble();
        }
    }

    public Command blow() {
        return startEnd(() -> setState(State.BLOW), () -> setState(State.STOP)).withName("BlowTrap");
    }
}

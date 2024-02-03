package frc.robot.subsystems.shooter.roller;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team6328.TunableNumber;
import frc.robot.subsystems.shooter.roller.RollerIO.RollerIOInputs;
import frc.robot.util.Util;

public class Roller extends SubsystemBase{

    private final RollerIO m_io;
    private final RollerIOInputs m_inputs = new RollerIOInputs();
    
    private State currentState = State.DO_NOTHING;

    private TunableNumber velocityThreshold = new TunableNumber("Roller/Velocity Threshold", 200.0);
    private TunableNumber currentThreshold = new TunableNumber("Roller/Current Threshold", 25.0);

    public Roller(RollerIO io){

        System.out.println("[Init] Creating Roller");
        this.m_io = io;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Roller");
             shuffleboardTab.addNumber("Velocity", () -> Util.truncate(getMotorRPM(), 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Current", () -> Util.truncate(getMotorCurrent(), 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Demand", () -> Util.truncate(currentState.getMotorVoltage(), 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Output", () -> Util.truncate(getMotorOutput(), 2))
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addBoolean("Is Stalled?", this::isStalled);

        shuffleboardTab.addString("Control State", () -> currentState.name());
        shuffleboardTab.addString("Command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "NONE");

    }

    @Override
    public void periodic(){
        m_io.updateInputs(m_inputs);
        m_io.setRollerVoltage(this.currentState.getMotorVoltage());

        Logger.processInputs("Roller", m_inputs);
        Logger.recordOutput("CurrentState", currentState.toString());
        Logger.recordOutput("isStalled", isStalled());

    }

    public void setRollerState(State desState) {
        this.currentState = desState;
    }

    public double getMotorCurrent() {
        return m_inputs.RollerCurrentAmps[0];
    }

    public double getMotorRPM() {
        return m_inputs.RollerVelocityRPM;
    }

    public double getMotorOutput() {
        return m_inputs.RollerAppliedVolts;
    }

    public boolean isStalled() {
        return Math.abs(m_inputs.RollerVelocityRPM) <= velocityThreshold.get()
                && m_inputs.RollerCurrentAmps[0] >= currentThreshold.get();
    }


    public enum State {
        

        DO_NOTHING(0.0), IDLE(1.0), 

        INTAKING_NOTE(10.0), EJECT_NOTE(-7.0), 
        
        HOLD_NOTE(2.0);
        

        private double motorVoltage;

        private State(double motorVoltage) {
            this.motorVoltage = motorVoltage;
        }

        public double getMotorVoltage() {
            return motorVoltage;
        }
    }

    public Command intakeNoteCommand() {
        return Commands
                .sequence(
                    setRollerStateCommand(State.INTAKING_NOTE), 
                    new WaitCommand(0.5),
                    waitForNote())
                .finallyDo(interrupted -> setRollerState(State.HOLD_NOTE));
    }

    public Command idleCommand() {
        return setRollerStateCommand(State.IDLE);
    }

    public Command scoreCone() {
        return Commands.sequence(setRollerStateCommand(State.EJECT_NOTE), new WaitCommand(0.5),
                setRollerStateCommand(State.IDLE));
    }

    public Command setRollerStateCommand(State state) {
        return new InstantCommand(() -> setRollerState(state));
    }

    public Command waitForNote(){
        return new WaitUntilCommand(this::isStalled);

    }

}

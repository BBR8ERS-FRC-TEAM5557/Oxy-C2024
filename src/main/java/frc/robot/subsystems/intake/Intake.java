package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team6328.TunableNumber;
import frc.robot.util.Util;

public class Intake extends SubsystemBase{

    private final IntakeIO m_io;
    private final IntakeIOInputs m_inputs = new IntakeIOInputs();
    
    private State currentState = State.DO_NOTHING;

    private TunableNumber velocityThreshold = new TunableNumber("Intake/Velocity Threshold", 200.0);
    private TunableNumber currentThreshold = new TunableNumber("Intake/Current Threshold", 25.0);

    public Intake(IntakeIO io){

        System.out.println("[Init] Creating Roller");
        this.m_io = io;

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
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
        m_io.setIntakeVoltage(this.currentState.getMotorVoltage());

        Logger.processInputs("Intake", m_inputs);
        Logger.recordOutput("CurrentState", currentState.toString());
        Logger.recordOutput("isStalled", isStalled());

    }

    public void setRollerState(State desState) {
        this.currentState = desState;
    }

    public double getMotorCurrent() {
        return m_inputs.IntakeCurrentAmps[0];
    }

    public double getMotorRPM() {
        return m_inputs.IntakeVelocityRPM;
    }

    public double getMotorOutput() {
        return m_inputs.IntakeAppliedVolts;
    }

    public boolean isStalled() {
        return Math.abs(m_inputs.IntakeVelocityRPM) <= velocityThreshold.get()
                && m_inputs.IntakeCurrentAmps[0] >= currentThreshold.get();
    }


    public enum State {
        

        DO_NOTHING(0.0), IDLE(1.0);
        

        private double motorVoltage;

        private State(double motorVoltage) {
            this.motorVoltage = motorVoltage;
        }

        public double getMotorVoltage() {
            return motorVoltage;
        }
    }




}

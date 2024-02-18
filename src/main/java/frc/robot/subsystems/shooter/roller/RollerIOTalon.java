package frc.robot.subsystems.shooter.roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlFrame;

import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import frc.lib.team5557.factory.TalonFactory;
import com.ctre.phoenix6.signals.ControlModeValue;

import static frc.robot.subsystems.intake.IntakeConstants.*;


public class RollerIOTalon implements RollerIO {

    private TalonFX motor;

    public RollerIOTalon() {
        System.out.println("[Init] Creating RollerIOTalon");
        motor = TalonFactory.createTalon(kRollerMotorConfiguration);
        //BurnManager.burnFlash(motor);
    }

    public void updateInputs(RollerIOInputs inputs) {
        inputs.RollerAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.RollerCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.RollerVelocityRPM = motor.getEncoder().getVelocity();
        inputs.RollerTempCelcius = new double[] {motor.getMotorTemperature()};
    }

    public void setRollerVoltage(double value) {
        motor.getPIDController().setReference(value, ControlType.kVoltage);
    }
}

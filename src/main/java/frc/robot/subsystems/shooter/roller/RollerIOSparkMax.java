package frc.robot.subsystems.shooter.roller;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class RollerIOSparkMax implements RollerIO {

    private CANSparkMax motor;
    //private TalonFX motor;
    
    public RollerIOSparkMax() {
        System.out.println("[Init] Creating RollerIOSparkMax");
        motor = SparkMaxFactory.createMotor(kRollerMotorConfiguration);
        BurnManager.burnFlash(motor);
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

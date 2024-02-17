package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax motor;
    
    public IntakeIOSparkMax() {
        System.out.println("[Init] Creating RollerIOSparkMax");
        motor = SparkMaxFactory.createMotor(kMasterIntakeMotorConfiguration);
        BurnManager.burnFlash(motor);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.IntakeAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.IntakeCurrentAmps = new double[] {motor.getOutputCurrent()};
        inputs.IntakeVelocityRPM = motor.getEncoder().getVelocity();
        inputs.IntakeTempCelcius = new double[] {motor.getMotorTemperature()};
    }

    public void setRollerVoltage(double value) {
        motor.getPIDController().setReference(value, ControlType.kVoltage);
    }
}

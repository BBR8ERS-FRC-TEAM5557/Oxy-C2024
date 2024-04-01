package frc.robot.subsystems.blower;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

import static frc.robot.subsystems.blower.BlowerConstants.*;

public class BlowerIOSparkMax implements BlowerIO {
    private CANSparkMax mMotor;

    public BlowerIOSparkMax() {
        System.out.println("[Init] Creating BlowerIOSparkMax");
        mMotor = SparkMaxFactory.createBrushed(kBlowerMotorConfiguration);
        BurnManager.burnFlash(mMotor);
    }

    @Override
    public void updateInputs(BlowerIOInputs inputs) {
        inputs.blowerAppliedVolts = mMotor.getAppliedOutput() * mMotor.getBusVoltage();
        inputs.blowerCurrentAmps = mMotor.getOutputCurrent();
    }

    @Override
    public void setBlowerVoltage(double volts) {
        mMotor.setVoltage(volts);
    }

    @Override
    public void setBrakeMode(boolean brake) {
        mMotor.setIdleMode(brake ? IdleMode.kBrake : IdleMode.kCoast);
    }
}

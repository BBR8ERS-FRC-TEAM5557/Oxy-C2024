package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

import static frc.robot.subsystems.intake.IntakeConstants.*;

public class IntakeIOSparkMax implements IntakeIO {

    private CANSparkMax mTopMotor;
    private CANSparkMax mBottomMotor;

    public IntakeIOSparkMax() {
        System.out.println("[Init] Creating IntakeIOSparkMax");
        mTopMotor = SparkMaxFactory.createNEO(kTopMotorConfiguration);
        mBottomMotor = SparkMaxFactory.createNEO(kBottomMotorConfiguration);
        BurnManager.burnFlash(mTopMotor);
        BurnManager.burnFlash(mBottomMotor);
    }

    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeTopPositionRotations = mTopMotor.getEncoder().getPosition() / kTopGearReduction;
        inputs.intakeTopVelocityRPM = mTopMotor.getEncoder().getVelocity() / kTopGearReduction;
        inputs.intakeTopAppliedVolts = mTopMotor.getAppliedOutput() * mTopMotor.getBusVoltage();
        inputs.intakeTopCurrentAmps = mTopMotor.getOutputCurrent();
        inputs.intakeTopTempCelcius = mTopMotor.getMotorTemperature();

        inputs.intakeBottomPositionRotations = mBottomMotor.getEncoder().getPosition() / kBottomGearReduction;
        inputs.intakeBottomVelocityRPM = mBottomMotor.getEncoder().getVelocity() / kBottomGearReduction;
        inputs.intakeBottomAppliedVolts = mBottomMotor.getAppliedOutput() * mBottomMotor.getBusVoltage();
        inputs.intakeBottomCurrentAmps = mBottomMotor.getOutputCurrent();
        inputs.intakeBottomTempCelcius = mBottomMotor.getMotorTemperature();
    }

    @Override
    public void setIntakeVoltage(double topvalue, double bottomvalue) {
        mTopMotor.setVoltage(topvalue);
        mBottomMotor.setVoltage(bottomvalue);
    }
}
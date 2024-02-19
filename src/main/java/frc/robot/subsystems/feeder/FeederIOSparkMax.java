package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.robot.Constants.RobotMap;

public class FeederIOSparkMax implements FeederIO {
    private final CANSparkMax mMotor;
    private final DigitalInput mBanner;

    public FeederIOSparkMax() {
        System.out.println("[Init] Creating FeederIOSparkMax");
        mMotor = SparkMaxFactory.createNEO(kMotorConfiguration);
        BurnManager.burnFlash(mMotor);

        mBanner = new DigitalInput(RobotMap.kFeederBanner);
    }

    public void updateInputs(FeederIOInputs inputs) {
        inputs.hasGamepiece = !mBanner.get();
        inputs.feederPositionRotations = mMotor.getEncoder().getPosition() / kGearReduction;
        inputs.feederVelocityRPM = mMotor.getEncoder().getVelocity() / kGearReduction;
        inputs.feederAppliedVolts = mMotor.getAppliedOutput() * mMotor.getBusVoltage();
        inputs.feederCurrentAmps = mMotor.getOutputCurrent();
        inputs.feederTempCelcius = mMotor.getMotorTemperature();
    }

    @Override
    public void setFeederVoltage(double value) {
        mMotor.setVoltage(value);
    }
}
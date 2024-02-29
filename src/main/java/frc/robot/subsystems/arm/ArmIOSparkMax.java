package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

public class ArmIOSparkMax implements ArmIO {

    private final CANSparkMax mLeader;
    private final CANSparkMax mFollower;

    private final RelativeEncoder mInternalEncoder;
    private final AbsoluteEncoder mAbsoluteEncoder;
    private final SparkPIDController mPid;

    public ArmIOSparkMax() {
        System.out.println("[Init] Creating ArmIOSparkMax");

        mLeader = SparkMaxFactory.createNEO(kLeaderMotorConfiguration);
        mFollower = SparkMaxFactory.createNEO(kFollowerMotorConfiguration);
        mInternalEncoder = mLeader.getEncoder();
        mAbsoluteEncoder = mLeader.getAbsoluteEncoder();
        mPid = mLeader.getPIDController();

        mAbsoluteEncoder.setZeroOffset(kAbsoluteEncoderOffset);
        mAbsoluteEncoder.setInverted(kAbsoluteEncoderInverted);
        mInternalEncoder.setPosition(mAbsoluteEncoder.getPosition() / kGearRatio);
        mPid.setFeedbackDevice(mAbsoluteEncoder);
        mFollower.follow(mLeader, true);

        BurnManager.burnFlash(mLeader);
        BurnManager.burnFlash(mFollower);

        SparkMaxFactory.configFramesDefault(mLeader);
        SparkMaxFactory.configFramesAbsoluteEncoderBoost(mLeader);
        SparkMaxFactory.configFramesDefault(mFollower);
        SparkMaxFactory.configFramesFollower(mFollower);
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        inputs.armAbsolutePositionDeg = Units.rotationsToDegrees(mAbsoluteEncoder.getPosition());
        inputs.armAbsoluteVelocityDegPerSec = Units.rotationsToDegrees(mAbsoluteEncoder.getVelocity());

        inputs.armInternalPositionDeg = Units.rotationsToDegrees(mInternalEncoder.getPosition() / kGearRatio);
        inputs.armInternalVelocityDegPerSec = Units.rotationsToDegrees(mInternalEncoder.getVelocity() / kGearRatio)
                / 60;
        inputs.armAppliedVolts = new double[] { mLeader.getAppliedOutput() * mLeader.getBusVoltage(),
                mFollower.getAppliedOutput() * mFollower.getBusVoltage() };
        inputs.armSupplyCurrentAmps = new double[] { mLeader.getOutputCurrent(), mFollower.getOutputCurrent() };
        inputs.armTempCelsius = new double[] { mLeader.getMotorTemperature(), mFollower.getMotorTemperature() };
    }

    @Override
    public void setVoltage(double volts) {
        mLeader.setVoltage(volts);
    }

    @Override
    public void setPercent(double percent) {
        mLeader.set(percent);
    }

    @Override
    public void setPosition(double degrees, double feedforward) {
        mPid.setReference(Units.degreesToRotations(degrees), ControlType.kPosition, 0, feedforward);
    }

    @Override
    public void setBrakeMode(boolean enable) {
        mLeader.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
        mFollower.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setSoftLimits(boolean enable) {
        mLeader.enableSoftLimit(SoftLimitDirection.kReverse, enable);
        mLeader.enableSoftLimit(SoftLimitDirection.kForward, enable);
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        mPid.setP(kP);
        mPid.setI(kI);
        mPid.setD(kD);
    }
}
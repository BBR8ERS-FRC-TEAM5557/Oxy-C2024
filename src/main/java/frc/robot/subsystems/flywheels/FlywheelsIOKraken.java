package frc.robot.subsystems.flywheels;

import static frc.robot.subsystems.flywheels.FlywheelsConstants.kGearReduction;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.kLeftMotorConfiguration;
import static frc.robot.subsystems.flywheels.FlywheelsConstants.kRightMotorConfiguration;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team5557.factory.TalonFactory;

public class FlywheelsIOKraken implements FlywheelsIO {
	/* Hardware */
	private final TalonFX mLeftMotor;
	private final TalonFX mRightMotor;

	/* Status Signals */
	private final StatusSignal<Double> mLeftPosition;
	private final StatusSignal<Double> mLeftVelocity;
	private final StatusSignal<Double> mLeftAppliedVolts;
	private final StatusSignal<Double> mLeftSupplyCurrent;
	private final StatusSignal<Double> mLeftTorqueCurrent;

	private final StatusSignal<Double> mRightPosition;
	private final StatusSignal<Double> mRightVelocity;
	private final StatusSignal<Double> mRightAppliedVolts;
	private final StatusSignal<Double> mRightSupplyCurrent;
	private final StatusSignal<Double> mRightTorqueCurrent;

	/* Control Signals */
	private final VelocityVoltage mLeftVelocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
	private final VelocityTorqueCurrentFOC mLeftVelocityControlFOC = new VelocityTorqueCurrentFOC(0)
			.withUpdateFreqHz(0);
	private final NeutralOut mLeftNeutral = new NeutralOut().withUpdateFreqHz(0);

	private final VelocityVoltage mRightVelocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
	private final VelocityTorqueCurrentFOC mRightVelocityControlFOC = new VelocityTorqueCurrentFOC(0)
			.withUpdateFreqHz(0);
	private final NeutralOut mRightNeutral = new NeutralOut().withUpdateFreqHz(0);

	public FlywheelsIOKraken() {
		System.out.println("[Init] Creating FlywheelIOKraken");

		mLeftMotor = TalonFactory.createTalon(kLeftMotorConfiguration);

		mLeftPosition = mLeftMotor.getPosition();
		mLeftVelocity = mLeftMotor.getVelocity();
		mLeftAppliedVolts = mLeftMotor.getMotorVoltage();
		mLeftSupplyCurrent = mLeftMotor.getSupplyCurrent();
		mLeftTorqueCurrent = mLeftMotor.getTorqueCurrent();

		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				mLeftPosition,
				mLeftVelocity,
				mLeftAppliedVolts,
				mLeftSupplyCurrent,
				mLeftTorqueCurrent);

		mRightMotor = TalonFactory.createTalon(kRightMotorConfiguration);

		mRightPosition = mRightMotor.getPosition();
		mRightVelocity = mRightMotor.getVelocity();
		mRightAppliedVolts = mRightMotor.getMotorVoltage();
		mRightSupplyCurrent = mRightMotor.getSupplyCurrent();
		mRightTorqueCurrent = mRightMotor.getTorqueCurrent();
		BaseStatusSignal.setUpdateFrequencyForAll(
				50.0,
				mRightPosition,
				mRightVelocity,
				mRightAppliedVolts,
				mRightSupplyCurrent,
				mRightTorqueCurrent);

	}

	public void updateInputs(FlywheelsIOInputs inputs) {
		inputs.hasCurrentControl = false;
		inputs.leftMotorConnected = BaseStatusSignal.refreshAll(
				mLeftPosition,
				mLeftVelocity,
				mLeftAppliedVolts,
				mLeftSupplyCurrent,
				mLeftTorqueCurrent)
				.isOK();
		inputs.rightMotorConnected = BaseStatusSignal.refreshAll(
				mRightPosition,
				mRightPosition,
				mRightPosition,
				mRightPosition,
				mRightPosition)
				.isOK();

		inputs.leftPositionRotations = mLeftPosition.getValueAsDouble();
		inputs.leftVelocityRpm = mLeftVelocity.getValueAsDouble() * 60.0 / kGearReduction;
		inputs.leftAppliedVolts = mLeftAppliedVolts.getValueAsDouble();
		inputs.leftSupplyCurrentAmps = mLeftSupplyCurrent.getValueAsDouble();
		inputs.leftTorqueCurrentAmps = mLeftTorqueCurrent.getValueAsDouble();

		inputs.rightPositionRotations = mRightPosition.getValueAsDouble();
		inputs.rightVelocityRpm = mRightVelocity.getValueAsDouble() * 60.0 / kGearReduction;
		inputs.rightAppliedVolts = mRightAppliedVolts.getValueAsDouble();
		inputs.rightSupplyCurrentAmps = mRightSupplyCurrent.getValueAsDouble();
		inputs.rightTorqueCurrentAmps = mRightTorqueCurrent.getValueAsDouble();
	}

	@Override
	public void runVoltage(double voltage) {
		mLeftMotor.setVoltage(voltage);
		mRightMotor.setVoltage(voltage);
	}

	@Override
	public void runVelocity(double velocity, double feedforward) {
		mLeftMotor.setControl(mLeftVelocityControl.withVelocity(velocity * kGearReduction).withFeedForward(feedforward));
		mRightMotor.setControl(mRightVelocityControl.withVelocity(velocity * kGearReduction).withFeedForward(feedforward));
	}

	@Override
	public void stop() {
		mLeftMotor.setControl(mLeftNeutral);
		mRightMotor.setControl(mRightNeutral);
	}

	@Override
	public void setPID(double kP, double kI, double kD) {
        var leftFeedbackConfig = new Slot0Configs();
        leftFeedbackConfig.kP = kP;
        leftFeedbackConfig.kI = kI;
        leftFeedbackConfig.kD = kD;
        mLeftMotor.getConfigurator().apply(leftFeedbackConfig, 0.01);

		var rightFeedbackConfig = new Slot0Configs();
        rightFeedbackConfig.kP = kP;
        rightFeedbackConfig.kI = kI;
        rightFeedbackConfig.kD = kD;
        mRightMotor.getConfigurator().apply(rightFeedbackConfig, 0.01);
	}

	@Override
	public void setBrakeMode(boolean enable) {
		mRightMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
		mLeftMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
	}
}

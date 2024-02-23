package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team5557.factory.TalonFactory;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class ModuleIOKrakenSparkMax implements ModuleIO {
    // Drive Hardware
    private final TalonFX mDriveMotor;

    // Angle Hardware
    private final CANSparkMax mAngleMotor;
    private final SparkPIDController mAngleMotorPID;
    private final RelativeEncoder mAngleMotorEncoder;
    private final AbsoluteEncoder mAbsoluteEncoder;

    // Status Signals
    private final StatusSignal<Double> drivePosition;
    private final StatusSignal<Double> driveVelocity;
    private final StatusSignal<Double> driveAppliedVolts;
    private final StatusSignal<Double> driveSupplyCurrent;
    private final StatusSignal<Double> driveTorqueCurrent;

    // Control Signals?
    private final VoltageOut driveVoltage = new VoltageOut(0).withUpdateFreqHz(0);
    private final VelocityVoltage driveVelocityControl = new VelocityVoltage(0).withUpdateFreqHz(0);
    private final TorqueCurrentFOC driveCurrent = new TorqueCurrentFOC(0).withUpdateFreqHz(0);
    private final NeutralOut driveNeutral = new NeutralOut().withUpdateFreqHz(0);

    // Misc.
    private double resetIteration;

    public ModuleIOKrakenSparkMax(int moduleNumber, int driveMotorID, int angleMotorID, Rotation2d angleOffset) {
        System.out.println("[Init] Creating ModuleIOKrakenSparkMax" + moduleNumber);

        // ANGLE MOTOR
        mAngleMotor = SparkMaxFactory.createNEO(angleMotorID, kAngleMotorConfiguration);
        mAngleMotorPID = mAngleMotor.getPIDController();
        mAngleMotorEncoder = mAngleMotor.getEncoder();
        mAbsoluteEncoder = mAngleMotor.getAbsoluteEncoder();

        System.out.println(mAbsoluteEncoder.setInverted(kAbsoluteEncoderInverted).toString());
        System.out.println(mAbsoluteEncoder.setInverted(kAbsoluteEncoderInverted).toString());
        System.out.println(mAbsoluteEncoder.setInverted(kAbsoluteEncoderInverted).toString());
        System.out.println(mAbsoluteEncoder.setInverted(kAbsoluteEncoderInverted).toString());
        mAbsoluteEncoder.setPositionConversionFactor(1.0);
        System.out.println(mAbsoluteEncoder.setZeroOffset(angleOffset.getRotations()));
        System.out.println(mAbsoluteEncoder.setZeroOffset(angleOffset.getRotations()));
        System.out.println(mAbsoluteEncoder.setZeroOffset(angleOffset.getRotations()));
        System.out.println(mAbsoluteEncoder.setZeroOffset(angleOffset.getRotations()));
        mAbsoluteEncoder.setAverageDepth(2);
        mAngleMotorPID.setPositionPIDWrappingMinInput(0.0);
        mAngleMotorPID.setPositionPIDWrappingMaxInput(kAngleGearReduction);
        mAngleMotorPID.setPositionPIDWrappingEnabled(true);
        BurnManager.burnFlash(mAngleMotor);
        SparkMaxFactory.configFramesDefault(mAngleMotor);
        SparkMaxFactory.configFramesAbsoluteEncoderBoost(mAngleMotor);

        // DRIVE MOTOR
        mDriveMotor = TalonFactory.createTalon(driveMotorID, kDriveMotorConfiguration);

        driveVelocity = mDriveMotor.getVelocity();
        driveAppliedVolts = mDriveMotor.getMotorVoltage();
        driveSupplyCurrent = mDriveMotor.getSupplyCurrent();
        driveTorqueCurrent = mDriveMotor.getTorqueCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent);

        drivePosition = mDriveMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, drivePosition);
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.hasCurrentControl = false;
        inputs.driveMotorConnected = BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent)
                .isOK();

        inputs.driveDistanceMeters = rotationsToMeters(
                drivePosition.getValueAsDouble(),
                kWheelCircumference,
                kDriveGearReduction);
        inputs.driveVelocityMetersPerSec = rotationsToMeters(
                driveVelocity.getValueAsDouble(),
                kWheelCircumference,
                kDriveGearReduction);
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveSupplyCurrentAmps = driveSupplyCurrent.getValueAsDouble();
        inputs.driveTorqueCurrentAmps = driveTorqueCurrent.getValueAsDouble();

        inputs.angleAbsolutePositionRad = getAbsoluteRotation().getRadians();
        inputs.angleInternalPositionRad = rotationsToRadians(mAngleMotorEncoder.getPosition(), kAngleGearReduction);
        inputs.angleInternalVelocityRadPerSec = rotationsToRadians(mAngleMotorEncoder.getVelocity(),
                kAngleGearReduction) / 60.0;
        inputs.angleAppliedVolts = mAngleMotor.getAppliedOutput() * mAngleMotor.getBusVoltage();
        inputs.angleSupplyCurrentAmps = mAngleMotor.getOutputCurrent();
        inputs.angleTempCelsius = mAngleMotor.getMotorTemperature();
    }

    @Override
    public void setDriveVoltage(double voltage) {
        mDriveMotor.setControl(driveVoltage.withOutput(voltage));
    }

    /** Run the drive motor at the specified velocity. */
    @Override
    public void setDriveVelocity(double velocity, double feedforward) {
        double rotationsPerSecond = metersToRotations(
                velocity,
                kWheelCircumference,
                kDriveGearReduction);

        mDriveMotor.setControl(driveVelocityControl.withVelocity(rotationsPerSecond).withFeedForward(feedforward));
    }

    @Override
    public void setAngleVoltage(double voltage) {
        mAngleMotorPID.setReference(voltage, ControlType.kVoltage);
    }

    /** Run the turn motor to the specified angle. */
    @Override
    public void setAnglePosition(double radians) {
        double desiredAngleRotations = radiansToRotations(radians, kAngleGearReduction);
        mAngleMotorPID.setReference(desiredAngleRotations, ControlType.kPosition);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        mDriveMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setAngleBrakeMode(boolean enable) {
        mAngleMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        mAngleMotorPID.setP(kP);
        mAngleMotorPID.setI(kI);
        mAngleMotorPID.setD(kD);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        var driveFeedbackConfig = new Slot0Configs();
        driveFeedbackConfig.kP = kP;
        driveFeedbackConfig.kI = kI;
        driveFeedbackConfig.kD = kD;
        mDriveMotor.getConfigurator().apply(driveFeedbackConfig, 0.01);
    }

    @Override
    public void stop() {
        mDriveMotor.setControl(driveNeutral);
        mAngleMotor.set(0.0);
    }

    @Override
    public boolean resetToAbsolute() {
        double absoluteAngle = getAbsoluteRotation().getRadians();
        return mAngleMotorEncoder
                .setPosition(radiansToRotations(absoluteAngle, kAngleGearReduction)) == REVLibError.kOk;
    }

    private Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromRotations(mAbsoluteEncoder.getPosition());
    }

}
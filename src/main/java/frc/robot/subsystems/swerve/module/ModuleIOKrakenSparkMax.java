package frc.robot.subsystems.swerve.module;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team5557.factory.TalonFactory;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class ModuleIOKrakenSparkMax implements ModuleIO {
    // Drive Hardware
    private final TalonFX m_driveMotor;

    // Angle Hardware
    private final CANSparkMax m_angleMotor;
    private final SparkPIDController m_angleMotorPID;
    private final RelativeEncoder m_angleMotorEncoder;
    private final AbsoluteEncoder m_absoluteEncoder;

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
        System.out.println("[Init] Creating ModuleIOSparkMax" + moduleNumber);

        // ANGLE MOTOR
        m_angleMotor = SparkMaxFactory.createNEO(angleMotorID, kAngleMotorConfiguration);
        m_angleMotorPID = m_angleMotor.getPIDController();
        m_angleMotorEncoder = m_angleMotor.getEncoder();
        m_absoluteEncoder = m_angleMotor.getAbsoluteEncoder();

        m_absoluteEncoder.setInverted(kAbsoluteEncoderInverted);
        m_absoluteEncoder.setPositionConversionFactor(1.0);
        m_absoluteEncoder.setZeroOffset(angleOffset.getRotations());
        m_absoluteEncoder.setAverageDepth(4); // ? idk what depth to put ://
        m_angleMotorPID.setPositionPIDWrappingMinInput(0.0);
        m_angleMotorPID.setPositionPIDWrappingMaxInput(kAngleGearReduction);
        m_angleMotorPID.setPositionPIDWrappingEnabled(true);
        BurnManager.burnFlash(m_angleMotor);
        int resetIteration = 0;
        while (resetIteration < 5)
            resetToAbsolute();

        // DRIVE MOTOR
        m_driveMotor = TalonFactory.createTalon(driveMotorID, kDriveMotorConfiguration);

        driveVelocity = m_driveMotor.getVelocity();
        driveAppliedVolts = m_driveMotor.getMotorVoltage();
        driveSupplyCurrent = m_driveMotor.getSupplyCurrent();
        driveTorqueCurrent = m_driveMotor.getTorqueCurrent();
        BaseStatusSignal.setUpdateFrequencyForAll(
                100.0,
                driveVelocity,
                driveAppliedVolts,
                driveSupplyCurrent,
                driveTorqueCurrent);

        drivePosition = m_driveMotor.getPosition();
        BaseStatusSignal.setUpdateFrequencyForAll(250.0, drivePosition);
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
        inputs.driveSupplyCurrentAmps = new double[] { driveSupplyCurrent.getValueAsDouble() };
        inputs.driveTorqueCurrentAmps = new double[] { driveTorqueCurrent.getValueAsDouble() };

        inputs.angleAbsolutePositionRad = getAbsoluteRotation().getRadians();
        inputs.angleInternalPositionRad = rotationsToRadians(m_angleMotorEncoder.getPosition(), kAngleGearReduction);
        inputs.angleInternalVelocityRadPerSec = rotationsToRadians(m_angleMotorEncoder.getVelocity(),
                kAngleGearReduction) / 60.0;
        inputs.angleAppliedVolts = m_angleMotor.getAppliedOutput() * m_angleMotor.getBusVoltage();
        inputs.angleSupplyCurrentAmps = new double[] { m_angleMotor.getOutputCurrent() };
        inputs.angleTempCelsius = new double[] { m_angleMotor.getMotorTemperature() };
    }

    @Override
    public void setDriveVoltage(double voltage) {
        m_driveMotor.setControl(driveVoltage.withOutput(voltage));
    }

    /** Run the drive motor at the specified velocity. */
    @Override
    public void setDriveVelocity(double velocity, double feedforward) {
        double rotationsPerSecond = metersToRotations(
                velocity,
                kWheelCircumference,
                kDriveGearReduction);

        m_driveMotor.setControl(driveVelocityControl.withVelocity(rotationsPerSecond).withFeedForward(feedforward));
    }

    @Override
    public void setAngleVoltage(double voltage) {
        if (m_angleMotorEncoder.getVelocity() < kAbsoluteResetMaxOmega) {
            if (++resetIteration >= kAbsoluteResetIterations) {
                resetIteration = 0;
                resetToAbsolute();
            }
        } else {
            resetIteration = 0;
        }

        m_angleMotorPID.setReference(voltage, ControlType.kVoltage);
    }

    /** Run the turn motor to the specified angle. */
    @Override
    public void setAnglePosition(double radians) {
        double desiredAngleRotations = radiansToRotations(radians, kAngleGearReduction);

        if (m_angleMotorEncoder.getVelocity() < kAbsoluteResetMaxOmega) {
            if (++resetIteration >= kAbsoluteResetIterations) {
                resetIteration = 0;
                resetToAbsolute();
            }
        } else {
            resetIteration = 0;
        }

        m_angleMotorPID.setReference(desiredAngleRotations, ControlType.kPosition);
    }

    /** Enable or disable brake mode on the drive motor. */
    @Override
    public void setDriveBrakeMode(boolean enable) {
        m_driveMotor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    }

    /** Enable or disable brake mode on the turn motor. */
    @Override
    public void setAngleBrakeMode(boolean enable) {
        m_angleMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
    }

    @Override
    public void setAnglePID(double kP, double kI, double kD) {
        m_angleMotorPID.setP(kP);
        m_angleMotorPID.setI(kI);
        m_angleMotorPID.setD(kD);
    }

    @Override
    public void setDrivePID(double kP, double kI, double kD) {
        var driveFeedbackConfig = new Slot0Configs();
        driveFeedbackConfig.kP = kP;
        driveFeedbackConfig.kI = kI;
        driveFeedbackConfig.kD = kD;
        m_driveMotor.getConfigurator().apply(driveFeedbackConfig, 0.01);
    }

    @Override
    public void stop() {
        m_driveMotor.setControl(driveNeutral);
        m_angleMotor.set(0.0);
    }

    @Override
    public boolean resetToAbsolute() {
        double absoluteAngle = getAbsoluteRotation().getRadians();
        m_angleMotorEncoder.setPosition(radiansToRotations(absoluteAngle, kAngleGearReduction));
        return true;
    }

    private Rotation2d getAbsoluteRotation() {
        return Rotation2d.fromRotations(m_absoluteEncoder.getPosition());
    }

}
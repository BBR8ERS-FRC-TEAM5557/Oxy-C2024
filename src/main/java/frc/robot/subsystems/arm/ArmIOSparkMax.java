package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team6328.TunableNumber;

public class ArmIOSparkMax implements ArmIO {

    private final CANSparkMax m_leader;
    private final CANSparkMax m_follower;

    private final RelativeEncoder m_encoder;
    private final SparkPIDController m_pid;
    private final ArmFeedforward m_feedforward;

    private final TunableNumber armkP = new TunableNumber("Arm/ArmkP", kArmkP);
    private final TunableNumber armkI = new TunableNumber("Arm/ArmkI", kArmkI);
    private final TunableNumber armkD = new TunableNumber("Arm/ArmkD", kArmkD);

    public ArmIOSparkMax() {
        System.out.println("[Init] Creating ArmIOSparkMax");
        m_leader = SparkMaxFactory.createNEO(kArmLeaderMotorConfiguration);
        m_follower = SparkMaxFactory.createNEO(kArmFollowerMotorConfiguration);

        m_encoder = m_leader.getEncoder();
        m_pid = m_leader.getPIDController();
        m_pid.setFeedbackDevice(m_encoder);
        m_encoder.setPosition(degreesToRotations(kEncoderHomePosition));
        BurnManager.burnFlash(m_leader);
        BurnManager.burnFlash(m_follower);

        SparkMaxFactory.configFramesLeaderOrFollower(m_leader);
        SparkMaxFactory.configFramesPositionBoost(m_leader);
        SparkMaxFactory.configFramesAbsoluteEncoderBoost(m_leader);

        SparkMaxFactory.configNonLeader(m_follower);
        SparkMaxFactory.configFramesPositionBoost(m_follower);


        m_feedforward = new ArmFeedforward(kArmkS, kArmkG, kArmkV, kArmkA);
    }

    /** Updates the set of loggable inputs. */
    public void updateInputs(ArmIOInputs inputs) {
        inputs.ArmInternalPositionDeg = rotationsToDegrees(m_encoder.getPosition());
        inputs.ArmInternalVelocityDegPerSec = rotationsToDegrees(m_encoder.getVelocity()) / 60.0;
        inputs.ArmAppliedVolts = m_leader.getAppliedOutput() * m_leader.getBusVoltage();
        inputs.ArmCurrentAmps = new double[] { m_leader.getOutputCurrent() };
        inputs.ArmTempCelsius = new double[] { m_leader.getMotorTemperature() };

        // update tunables
        if (armkP.hasChanged(armkP.hashCode()) || armkI.hasChanged(armkI.hashCode())
                || armkD.hasChanged(armkD.hashCode())) {
            m_pid.setP(armkP.get());
            m_pid.setI(armkI.get());
            m_pid.setD(armkD.get());
        }
    }

    /** Run the Arm open loop at the specified voltage. */
    public void setVoltage(double volts) {
        m_pid.setReference(volts, ControlType.kVoltage);
    }

    public void setPercent(double percent) {
        m_pid.setReference(percent, ControlType.kDutyCycle);
    }

    public void setAngleDegrees(double targetAngleDegrees, double targetVelocityDegreesPerSec) {
        double ff = m_feedforward.calculate(targetAngleDegrees, targetVelocityDegreesPerSec);
        targetAngleDegrees = constrainDegrees(targetAngleDegrees);
        double targetRotation = degreesToRotations(targetAngleDegrees);
        m_pid.setReference(targetRotation, ControlType.kPosition, 0, ff);
    }

    public void resetSensorPosition(double angleDegrees) {
        m_encoder.setPosition(degreesToRotations(angleDegrees));
    }

    public void brakeOff() {
        m_leader.setIdleMode(IdleMode.kCoast);
    }

    public void brakeOn() {
        m_leader.setIdleMode(IdleMode.kBrake);
    }

    public void shouldEnableUpperLimit(boolean value) {
        m_leader.enableSoftLimit(SoftLimitDirection.kForward, value);
    }

    public void shouldEnableLowerLimit(boolean value) {
        m_leader.enableSoftLimit(SoftLimitDirection.kReverse, value);
    }
}
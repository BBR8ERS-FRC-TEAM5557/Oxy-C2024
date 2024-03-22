package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.kArmA;
import static frc.robot.subsystems.arm.ArmConstants.kArmD;
import static frc.robot.subsystems.arm.ArmConstants.kArmG;
import static frc.robot.subsystems.arm.ArmConstants.kArmI;
import static frc.robot.subsystems.arm.ArmConstants.kArmP;
import static frc.robot.subsystems.arm.ArmConstants.kArmS;
import static frc.robot.subsystems.arm.ArmConstants.kArmV;
import static frc.robot.subsystems.arm.ArmConstants.kCruiseVelocity;
import static frc.robot.subsystems.arm.ArmConstants.kMaxAngle;
import static frc.robot.subsystems.arm.ArmConstants.kMinAngle;
import static frc.robot.subsystems.arm.ArmConstants.kPadding;
import static frc.robot.subsystems.arm.ArmConstants.kTimeToCruise;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.RobotStateEstimator;
import frc.robot.subsystems.arm.ArmIO.ArmIOInputs;
import frc.robot.util.Util;
import lombok.RequiredArgsConstructor;

public class Arm extends SubsystemBase {
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", kArmP);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", kArmI);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", kArmD);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", kArmS);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Arm/kG", kArmG);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", kArmV);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", kArmA);

    private static final LoggedTunableNumber cruiseVelocity = new LoggedTunableNumber("Arm/Velocity", kCruiseVelocity);
    private static final LoggedTunableNumber timeToCruise = new LoggedTunableNumber("Arm/TimeToCruise",
            kTimeToCruise);

    private static final LoggedTunableNumber stowDegrees = new LoggedTunableNumber("Superstructure/ArmStowDegrees",
            157.0);
    private static final LoggedTunableNumber intakeDegrees = new LoggedTunableNumber("Superstructure/ArmIntakeDegrees",
            157.0);

    private static final LoggedTunableNumber ampDegrees = new LoggedTunableNumber("Superstructure/ArmAmpDegrees",
            250.0);
    private static final LoggedTunableNumber trapDegrees = new LoggedTunableNumber("Superstructure/ArmTrapDegrees",
            260.0);
    private static final LoggedTunableNumber fenderShotDegrees = new LoggedTunableNumber(
            "Superstructure/ArmFenderShotDegrees",
            156.5);
    private static final LoggedTunableNumber customShotDegrees = new LoggedTunableNumber(
            "Superstructure/ArmCustomShotDegrees",
            174.5);
    private static final LoggedTunableNumber climbPrepDegrees = new LoggedTunableNumber(
            "Superstructure/ArmClimbPrepDegrees",
            270.0);
    private static final LoggedTunableNumber climbRetractDegrees = new LoggedTunableNumber(
            "Superstructure/ArmClimbRetractDegrees",
            157.0);

    private final ArmIO mIO;
    private final ArmIOInputs mInputs = new ArmIOInputs();
    private State mState = State.STOW;

    private TrapezoidProfile mMotionProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(kCruiseVelocity, kCruiseVelocity / kTimeToCruise));
    private TrapezoidProfile.State mSetpointState = new TrapezoidProfile.State(stowDegrees.get(), 0.0);
    private ArmFeedforward mFeedforward = new ArmFeedforward(kArmS, kArmG, kArmV, kArmA);

    public Arm(ArmIO io) {
        System.out.println("[Init] Creating Arm");
        this.mIO = io;
    }

    @Override
    public void periodic() {
        mIO.updateInputs(mInputs);
        Logger.processInputs("Arm", mInputs);

        // Update controllers
        LoggedTunableNumber.ifChanged(
                hashCode(), () -> mIO.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> mFeedforward = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
                kS,
                kG,
                kV,
                kA);
        LoggedTunableNumber.ifChanged(
                hashCode(),
                () -> mMotionProfile = new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(cruiseVelocity.get(),
                                cruiseVelocity.get() / timeToCruise.get())),
                cruiseVelocity,
                timeToCruise);

        if (mState == State.CHARACTERIZING) {
            // TODO:
        } else {
            double goal = MathUtil.clamp(mState.getDegrees(), kMinAngle, kMaxAngle);
            mSetpointState = mMotionProfile.calculate(
                    Robot.defaultPeriodSecs,
                    mSetpointState, // new TrapezoidProfile.State(mInputs.armAbsolutePositionDeg,
                                    // mInputs.armAbsoluteVelocityDegPerSec),
                    new TrapezoidProfile.State(goal, 0.0));

            double ff = mFeedforward.calculate(Units.degreesToRadians(180.0 - mSetpointState.position), mSetpointState.velocity); // (180 - setpoint) accounts for our weird encoder offset
            mIO.setPosition(mSetpointState.position, ff);
        }

        if (DriverStation.isDisabled()) {
            mIO.stop();
            mSetpointState = new TrapezoidProfile.State(mInputs.armAbsolutePositionDeg, 0);
        }

        Logger.recordOutput("Arm/SetpointAngle", mSetpointState.position);
        Logger.recordOutput("Arm/SetpointVelocity", mSetpointState.velocity);
        Logger.recordOutput("Arm/StateAngle", mState.getDegrees());
        Logger.recordOutput("Arm/State", mState);
    }

    @RequiredArgsConstructor
    public enum State {
        STOW(() -> stowDegrees.get()),
        FLOOR_INTAKE(() -> intakeDegrees.get()),
        AMP(() -> ampDegrees.get()),
        PASS(() -> 165.0), //potentially add supplier thing
        FENDER_SHOT(() -> fenderShotDegrees.get()),
        AIM(() -> RobotStateEstimator.getInstance().getAimingParameters().armAngle().getDegrees()),
        TRAP(() -> trapDegrees.get()),
        CLIMB_PREP(() -> climbPrepDegrees.get()),
        CLIMB_RETRACT(() -> climbRetractDegrees.get()),
        CUSTOM(() -> customShotDegrees.get()),
        CHARACTERIZING(() -> 0.0);

        private final DoubleSupplier armSetpointSupplier;

        private double getDegrees() {
            return armSetpointSupplier.getAsDouble();
        }
    }

    private void setState(State state) {
        this.mState = state;
    }

    public Command intake() {
        return startEnd(() -> setState(State.FLOOR_INTAKE), () -> setState(State.STOW));
    }

    public Command amp() {
        return startEnd(() -> setState(State.AMP), () -> setState(State.STOW));
    }

    public Command trap() {
        return runOnce(() -> setState(State.TRAP));
    }

    public Command aimFender() {
        return startEnd(() -> setState(State.FENDER_SHOT), () -> setState(State.STOW));
    }

    public Command pass() {
        return startEnd(() -> setState(State.PASS), () -> setState(State.STOW));
    }

    public Command aimCustom() {
        return startEnd(() -> setState(State.CUSTOM), () -> setState(State.STOW));
    }

    public Command aim() {
        return startEnd(() -> setState(State.AIM), () -> setState(State.STOW));
    }

    public Command prepClimb() {
        return runOnce(() -> setState(State.CLIMB_PREP));
    }

    public Command retractClimb() {
        return runOnce(() -> setState(State.CLIMB_RETRACT));
    }

    public Command jog(DoubleSupplier percent) {
        return new RunCommand(() -> mIO.setPercent(percent.getAsDouble()), this);
    }

    public void stop() {
        mIO.stop();
    }

    public Command forceCoast() {
        return startEnd(() -> mIO.setBrakeMode(false), () -> mIO.setBrakeMode(true))
                .ignoringDisable(true);
    }

    public Rotation2d getSetpoint() {
        return Rotation2d.fromRadians(mState.getDegrees());
    }

    public boolean atGoal() {
        return Util.epsilonEquals(mInputs.armAbsolutePositionDeg, mState.getDegrees(), kPadding);
    }
}
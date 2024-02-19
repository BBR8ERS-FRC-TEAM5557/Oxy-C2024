package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.auto.SystemsCheckManager.SwerveModuleSystemCheckRequest;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIO.GyroIOInputs;
import frc.robot.subsystems.swerve.module.Module;
import frc.robot.subsystems.swerve.module.ModuleIO;
import frc.robot.subsystems.swerve.util.KinematicLimits;
import frc.robot.subsystems.swerve.util.SwerveSetpoint;
import frc.robot.subsystems.swerve.util.SwerveSetpointGenerator;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.Util;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final GyroIO m_gyroIO;
  private final GyroIOInputs m_gyroInputs = new GyroIOInputs();

  private final Module[] m_modules = new Module[4]; // FL, FR, BL, BR

  public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(kSwerveModuleLocations);
  private SwerveSetpointGenerator m_setpointGenerator = new SwerveSetpointGenerator(m_kinematics,
      kSwerveModuleLocations);
  private ControlMode m_mode = ControlMode.VELOCITY;
  private KinematicLimits m_kinematicLimits = kUncappedLimits;
  private ChassisSpeeds m_desChassisSpeeds = new ChassisSpeeds();
  private SwerveSetpoint m_swerveSetpoint = new SwerveSetpoint(
      m_desChassisSpeeds,
      new SwerveModuleState[] {
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState(),
          new SwerveModuleState()
      });
  private Twist2d m_fieldVelocity = new Twist2d();
  private int m_systemCheckModuleNumber = 0;
  private SwerveModuleSystemCheckRequest m_systemCheckState = SwerveModuleSystemCheckRequest.DO_NOTHING;
  private double m_characterizationVolts = 0.0;

  public enum ControlMode {
    X_OUT,
    OPEN_LOOP,
    VELOCITY,
    PATH_FOLLOWING,
    CHARACTERIZATION,
    SYSTEMS_CHECK
  }

  public Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    System.out.println("[Init] Creating Swerve");
    this.m_gyroIO = gyroIO;
    m_modules[0] = new Module(flModuleIO, 0);
    m_modules[1] = new Module(frModuleIO, 1);
    m_modules[2] = new Module(blModuleIO, 2);
    m_modules[3] = new Module(brModuleIO, 3);

    ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Swerve");
    shuffleboardTab
        .addNumber("Heading", () -> Util.truncate(getGyroYaw().getDegrees(), 2))
        .withWidget(BuiltInWidgets.kGyro);
    shuffleboardTab
        .addNumber(
            "Velocity",
            () -> Util.truncate(Math.hypot(getFieldVelocity().dx, getFieldVelocity().dy), 2))
        .withWidget(BuiltInWidgets.kGraph);

    shuffleboardTab
        .addNumber("Velocity Kinematic Limit", () -> getKinematicLimit().maxLinearVelocity())
        .withWidget(BuiltInWidgets.kNumberBar);
    shuffleboardTab.addString("Control Mode", () -> getControlMode().name());
    shuffleboardTab.addString(
        "Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "NONE");

    SmartDashboard.putData("Swerve Viewable", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");
        builder.addDoubleProperty("Front Left Angle", () -> m_modules[0].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_modules[0].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Front Right Angle", () -> m_modules[1].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_modules[1].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Left Angle", () -> m_modules[2].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_modules[2].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Back Right Angle", () -> m_modules[3].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_modules[3].getState().speedMetersPerSecond, null);
        builder.addDoubleProperty("Robot Angle",
            () -> (RobotStateEstimator.isRedAlliance)
                ? getGyroYaw().rotateBy(Rotation2d.fromDegrees(180.0)).getRadians()
                : getGyroYaw().getRadians(),
            null);
      }
    });
  }

  @Override
  public void periodic() {
    m_gyroIO.updateInputs(m_gyroInputs);
    Logger.processInputs(kSubsystemName + "/Gyro", m_gyroInputs);
    for (var module : m_modules) {
      module.updateAndProcessInputs();
    }

    // Run modules
    if (DriverStation.isDisabled()) {
      // Stop moving while disabled
      for (var module : m_modules) {
        module.stop();
      }

      // Clear setpoint logs
      Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
      Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});

    } else if (m_mode == ControlMode.X_OUT) {
      for (var module : m_modules) {
        module.runSetpoint(null, true, true);
      }
    } else if (m_mode == ControlMode.CHARACTERIZATION) {
      // Run in characterization mode
      for (var module : m_modules) {
        module.setVoltageForCharacterization(m_characterizationVolts);
      }

      // Clear setpoint logs
      Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
      Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});

    } else if (m_mode == ControlMode.SYSTEMS_CHECK) {
      for (int i = 0; i < m_modules.length; i++) {
        if (i == m_systemCheckModuleNumber) {
          if (m_systemCheckState.isSetpointCheck()) {
            m_modules[i].runSetpoint(m_systemCheckState.getSwerveModuleState(), true, true);
          } else {
            m_modules[i].setVoltageForAzimuthCheck(m_systemCheckState.getSteerVoltage());
          }
        } else {
          m_modules[i].stop();
        }
      }
      /* FIX THIS */

      // Clear setpoint logs
      Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", new double[] {});
      Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", new double[] {});
    } else {
      // Calculate module setpoints
      var setpointTwist = new Pose2d()
          .log(
              new Pose2d(
                  m_desChassisSpeeds.vxMetersPerSecond * Robot.defaultPeriodSecs,
                  m_desChassisSpeeds.vyMetersPerSecond * Robot.defaultPeriodSecs,
                  new Rotation2d(
                      m_desChassisSpeeds.omegaRadiansPerSecond * Robot.defaultPeriodSecs * 4)));

      m_desChassisSpeeds = new ChassisSpeeds(
          setpointTwist.dx / Robot.defaultPeriodSecs,
          setpointTwist.dy / Robot.defaultPeriodSecs,
          m_desChassisSpeeds.omegaRadiansPerSecond);



      m_swerveSetpoint = m_setpointGenerator.generateSetpoint(
          kModuleLimits, m_swerveSetpoint, m_desChassisSpeeds, Robot.defaultPeriodSecs);

      // m_swerveSetpoint.moduleStates = m_kinematics.toSwerveModuleStates(adjustedSpeeds);

      // Send setpoints to modules
      SwerveModuleState[] optimizedStates = new SwerveModuleState[4];

      if (m_mode == ControlMode.PATH_FOLLOWING) {
        for (int i = 0; i < 4; i++) {
          optimizedStates[i] = m_modules[i].runSetpoint(m_swerveSetpoint.moduleStates()[i], false, false);
        }
      } else {
        for (int i = 0; i < 4; i++) {
          optimizedStates[i] = m_modules[i].runSetpoint(m_swerveSetpoint.moduleStates()[i], true, false);
        }
      }

      // Log setpoint states
      Logger.recordOutput(kSubsystemName + "/ModuleStates/Setpoints", m_swerveSetpoint.moduleStates());
      Logger.recordOutput(kSubsystemName + "/ModuleStates/SetpointsOptimized", optimizedStates);
    }

    // Log measured states
    SwerveModuleState[] measuredStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      measuredStates[i] = m_modules[i].getState();
    }

    Logger.recordOutput(kSubsystemName + "/ModuleStates/Measured", measuredStates);
    // Get measured positions
    SwerveModulePosition[] measuredPositions = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      measuredPositions[i] = m_modules[i].getPosition();
    }

    // Update Pose Estimator
    if (m_gyroInputs.connected) {
      RobotStateEstimator.getInstance().addDriveData(getGyroYaw(), measuredPositions);
    } else {
      RobotStateEstimator.getInstance().addDriveData(measuredPositions);
    }

    // Update field velocity
    ChassisSpeeds chassisSpeeds = m_kinematics.toChassisSpeeds(measuredStates);
    Translation2d linearFieldVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond)
        .rotateBy(getGyroYaw());
    m_fieldVelocity = new Twist2d(
        linearFieldVelocity.getX(),
        linearFieldVelocity.getY(),
        m_gyroInputs.connected
            ? m_gyroInputs.yawVelocityRadPerSec
            : chassisSpeeds.omegaRadiansPerSecond);

    Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VX", m_fieldVelocity.dx);
    Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VY", m_fieldVelocity.dy);
    Logger.recordOutput(kSubsystemName + "/ChassisSpeeds/VTHETA", m_fieldVelocity.dtheta);
    Logger.recordOutput(kSubsystemName + "/ControlMode", m_mode.toString());
    Logger.recordOutput(kSubsystemName + "/KinematicLimits", m_kinematicLimits.toString());
  }

  public void drive(ChassisSpeeds desSpeed, ControlMode desMode) {
    m_desChassisSpeeds = desSpeed;
    m_mode = desMode;
  }

  public void driveOpenLoop(ChassisSpeeds desSpeed) {
    this.drive(desSpeed, ControlMode.OPEN_LOOP);
  }

  public void driveVelocity(ChassisSpeeds desSpeed) {
    this.drive(desSpeed, ControlMode.VELOCITY);
  }

  public void drivePath(ChassisSpeeds desSpeed) {
    if (m_mode != ControlMode.PATH_FOLLOWING) {
      setKinematicLimits(kAutoLimits);
    }
    this.drive(desSpeed, ControlMode.PATH_FOLLOWING);
  }

  public void runModuleCheck(int moduleNumber, SwerveModuleSystemCheckRequest state) {
    if (m_mode != ControlMode.SYSTEMS_CHECK) {
      m_mode = ControlMode.SYSTEMS_CHECK;
    }
    m_systemCheckState = state;
    m_systemCheckModuleNumber = moduleNumber;
  }

  public void stop() {
    this.drive(new ChassisSpeeds(), ControlMode.OPEN_LOOP);
  }

  public void stopWithX() {
    this.drive(new ChassisSpeeds(), ControlMode.X_OUT);
  }

  public Rotation2d getGyroYaw() {
    return m_gyroInputs.connected
        ? m_gyroInputs.yawPosition
        : RobotStateEstimator.getInstance().getPose().getRotation();
  }

  public SwerveSetpoint getSwerveSetpoint() {
    return m_swerveSetpoint;
  }

  public Twist2d getFieldVelocity() {
    return m_fieldVelocity;
  }

  public KinematicLimits getKinematicLimit() {
    return m_kinematicLimits;
  }

  public ControlMode getControlMode() {
    return m_mode;
  }

  public boolean isUnderKinematicLimit(KinematicLimits limits) {
    return Math.hypot(getFieldVelocity().dx, getFieldVelocity().dy) < limits.maxLinearVelocity();
  }

  public void setKinematicLimits(KinematicLimits limits) {
    m_kinematicLimits = limits;
  }
}

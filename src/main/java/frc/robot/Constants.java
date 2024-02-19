package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.team5557.util.CANDeviceId;

public class Constants {

  public static boolean kIsReal = Robot.isReal();
  public static boolean kTuningMode = true;

  public class RobotMap {
    public static final int kPigeon = 5;

    public static final int kFLDriveMotor = 12;
    public static final int kFLTurnMotor = 21;
    public static final Rotation2d kFLOffset = Rotation2d.fromDegrees(164.79);

    public static final int kFRDriveMotor = 14;
    public static final int kFRTurnMotor = 22;
    public static final Rotation2d kFROffset = Rotation2d.fromDegrees(53.73);

    public static final int kBLDriveMotor = 13;
    public static final int kBLTurnMotor = 23;
    public static final Rotation2d kBLOffset = Rotation2d.fromDegrees(165.61);

    public static final int kBRDriveMotor = 15;
    public static final int kBRTurnMotor = 24;
    public static final Rotation2d kBROffset = Rotation2d.fromDegrees(326.1);

    public static final int kIntakeBottomMotor = 27;
    public static final int kIntakeTopMotor = 28;

    public static final int kFeederMotor = 0;
    public static final int kFeederBanner = 0;

    public static final int kFlywheelsLeftMotor = 0;
    public static final int kFlywheelRightMotor = 0;

    public static final int kArmLeaderMotor = 0;
    public static final int kArmFollowerMotor = 0;
  }
}

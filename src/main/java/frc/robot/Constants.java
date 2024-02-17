package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;

public class Constants {

  public static boolean kIsReal = Robot.isReal();
  public static boolean kTuningMode = true;

  public class RobotMap {
    public static final int kFLDriveMotor = 21;
    public static final int kFLTurnMotor = 22;
    public static final Rotation2d kFLOffset = Rotation2d.fromDegrees(-0.0);

    public static final int kFRDriveMotor = 27;
    public static final int kFRTurnMotor = 28;
    public static final Rotation2d kFROffset = Rotation2d.fromDegrees(-0.0);

    public static final int kBLDriveMotor = 23;
    public static final int kBLTurnMotor = 24;
    public static final Rotation2d kBLOffset = Rotation2d.fromDegrees(-0.0);

    public static final int kBRDriveMotor = 25;
    public static final int kBRTurnMotor = 26;
    public static final Rotation2d kBROffset = Rotation2d.fromDegrees(-0.0);

    //public static final int kShooterMotor = 0;
    //public static final int kClimbMotor = 0;
    public static final int kIntakeMotor = 0;
    public static final int kRollerMotor = 0;
    public static final int kWristMotor = 0;

    // public static final int kLedsDIO = 9;
  }
}

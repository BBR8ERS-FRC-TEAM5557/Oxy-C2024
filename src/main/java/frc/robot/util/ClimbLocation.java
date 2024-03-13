package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import lombok.RequiredArgsConstructor;

@RequiredArgsConstructor
public enum ClimbLocation {
    STAGE_LEFT(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(-60.0))),
    STAGE_CENTER(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180.0))),
    STAGE_RIGHT(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(60.0)));

    private final Pose2d pose;

    public Pose2d getPose() {
        return pose;
    }
}
// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.swerve.SwerveConstants;

public class AprilTagVisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.5;
    public static final double zMargin = 0.75;
    public static final double xyStdDevCoefficient = 0.05; // was 0.005
    public static final double thetaStdDevCoefficient = 0.01;

    public static final double[] stdDevFactors = new double[] { 1.0, 1.0 };

    public static final Pose3d[] cameraPoses = new Pose3d[] {
        //LEFT SHOOTER
            new Pose3d(
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(0.0),
                    Units.inchesToMeters(0.0),
                    new Rotation3d(Units.degreesToRadians(0.0), Units.degreesToRadians(0.0), 0.0) // -28.125
                            .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))), //180 - 10

        //RIGHT SHOOTER
            new Pose3d(
                    Units.inchesToMeters(-(7.823 + SwerveConstants.kTrueChassisCenterOffset)),
                    Units.inchesToMeters(-10.189),
                    Units.inchesToMeters(9.17),
                    new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0)
                            .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))),

        //LEFT SHOOTER
            new Pose3d(
                    Units.inchesToMeters(-(7.823 + SwerveConstants.kTrueChassisCenterOffset)),
                    Units.inchesToMeters(10.189),
                    Units.inchesToMeters(9.17),
                    new Rotation3d(Units.degreesToRadians(180 - 28.125), Units.degreesToRadians(0.0), 0.0) // -28.125
                            .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0)))), //180 - 10

        //RIGHT SHOOTER
            new Pose3d(
                    Units.inchesToMeters(-(7.823 + SwerveConstants.kTrueChassisCenterOffset)),
                    Units.inchesToMeters(-10.189),
                    Units.inchesToMeters(9.17),
                    new Rotation3d(0.0, Units.degreesToRadians(0.0), 0.0)
                            .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(0.0))))
    };

    public static final String[] instanceNames = { "leftShooter", "rightShooter" };

    public static final String[] cameraStreamIds = new String[] {
            "/dev/v4l/by-path/platform-fc800000.usb-usb-0:1:1.0-video-index0",
            "/dev/v4l/by-path/platform-fc880000.usb-usb-0:1:1.0-video-index0"
    };
}
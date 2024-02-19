package frc.robot.subsystems.swerve.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.DriveMotionPlanner;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.Util;

public class TeleopDrive extends Command {
    private final Swerve swerve;

    private final DoubleSupplier m_translationXSupplier;
    private final DoubleSupplier m_translationYSupplier;
    private final DoubleSupplier m_rotationSupplier;
    private final DoubleSupplier m_aimbotXSupplier;
    private final DoubleSupplier m_aimbotYSupplier;

    public TeleopDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier aimbotXSupplier, DoubleSupplier aimbotYSupplier) {
        this.swerve = RobotContainer.m_swerve;
        this.m_translationXSupplier = translationXSupplier;
        this.m_translationYSupplier = translationYSupplier;
        this.m_rotationSupplier = rotationSupplier;
        this.m_aimbotXSupplier = aimbotXSupplier;
        this.m_aimbotYSupplier = aimbotYSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var limit = swerve.getKinematicLimit();
        Rotation2d driveRotation = swerve.getGyroYaw();

        double rotationalVelocity = m_rotationSupplier.getAsDouble() * limit.maxLinearVelocity();
        double xVelocity = m_translationXSupplier.getAsDouble() * limit.maxLinearVelocity();
        double yVelocity = m_translationYSupplier.getAsDouble() * limit.maxLinearVelocity();

        double[] rightJoyPolarCoordinate = Util.toPolarCoordinate(m_aimbotYSupplier.getAsDouble(),
                m_aimbotXSupplier.getAsDouble());
        double r = Util.scaledDeadband(rightJoyPolarCoordinate[0], 1.0, 0.15);
        double theta = Units.radiansToDegrees(rightJoyPolarCoordinate[1]);

        if (r > 0.8) {
            theta /= 45;
            theta = Math.round(theta) * 45;
        }

        if (RobotStateEstimator.isRedAlliance) {
            theta = theta + 180.0;
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        if (r > 0.05) {
            rotationalVelocity = DriveMotionPlanner.calculateSnap(Rotation2d.fromDegrees(theta));
        }

        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                Util.clamp(rotationalVelocity, -limit.maxAngularVelocity(), limit.maxAngularVelocity()),
                driveRotation);

        swerve.driveOpenLoop(velocity);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

}

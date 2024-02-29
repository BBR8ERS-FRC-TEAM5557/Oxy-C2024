package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.DriveMotionPlanner;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.RobotStateEstimator;
import frc.robot.util.Util;

public class TeleopDrive extends Command {
    private final Swerve swerve;

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;
    private final DoubleSupplier mAimbotXSupplier;
    private final DoubleSupplier mAimbotYSupplier;
    private final BooleanSupplier mAutoaimSupplier;
    private final BooleanSupplier mWantsSnapSupplier;

    private LoggedTunableNumber headingPadding = new LoggedTunableNumber("Aiming/HeadingPaddingDeg", 1.0);
    private boolean atHeadingGoal = false;

    public TeleopDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier aimbotXSupplier, DoubleSupplier aimbotYSupplier,
            BooleanSupplier autoAimSupplier, BooleanSupplier wantsSnapSupplier) {
        this.swerve = RobotContainer.mSwerve;
        this.mTranslationXSupplier = translationXSupplier;
        this.mTranslationYSupplier = translationYSupplier;
        this.mRotationSupplier = rotationSupplier;
        this.mAimbotXSupplier = aimbotXSupplier;
        this.mAimbotYSupplier = aimbotYSupplier;
        this.mAutoaimSupplier = autoAimSupplier;
        this.mWantsSnapSupplier = wantsSnapSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var limit = swerve.getKinematicLimits();
        Rotation2d driveRotation = RobotStateEstimator.getInstance().getPose().getRotation();

        double rotationalVelocity = mRotationSupplier.getAsDouble() * limit.maxLinearVelocity();
        double xVelocity = mTranslationXSupplier.getAsDouble() * limit.maxLinearVelocity();
        double yVelocity = mTranslationYSupplier.getAsDouble() * limit.maxLinearVelocity();

        double[] rightJoyPolarCoordinate = Util.toPolarCoordinate(mAimbotYSupplier.getAsDouble(),
                mAimbotXSupplier.getAsDouble());
        double r = Util.scaledDeadband(rightJoyPolarCoordinate[0], 1.0, 0.15);
        Rotation2d theta = Rotation2d.fromRadians(rightJoyPolarCoordinate[1]);

        // if (r > 0.8) {
        //     var mod = theta.getDegrees() / 45;
        //     theta = Rotation2d.fromDegrees(Math.round(mod) * 45);
        // }
        if (mWantsSnapSupplier.getAsBoolean()) {
            var mod = theta.getDegrees() / 45;
            theta = Rotation2d.fromDegrees(Math.round(mod) * 45);
        }

        if (AllianceFlipUtil.shouldFlip()) {
            theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
            //theta = AllianceFlipUtil.apply(theta);
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        boolean wantsAutoAim = false;//mAutoaimSupplier.getAsBoolean();
        if (r > 0.05 || wantsAutoAim) {
            theta = wantsAutoAim ? theta : theta; // fix for vehicle stuff from 6328
            rotationalVelocity = DriveMotionPlanner.calculateSnap(theta);
            atHeadingGoal = Util.epsilonEquals(theta.getDegrees(), driveRotation.getDegrees(), headingPadding.getAsDouble());
        } else {
            theta = Rotation2d.fromDegrees(0.0);
        }

        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                Util.clamp(rotationalVelocity, -limit.maxAngularVelocity(), limit.maxAngularVelocity()),
                driveRotation);

        swerve.driveOpenLoop(velocity);

        Logger.recordOutput("RobotState/Aiming/HeadingGoal", theta);
        Logger.recordOutput("RobotState/Aiming/atHeadingGoal", theta);
    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    public boolean atHeadingGoal() {
        return atHeadingGoal;
    }

}

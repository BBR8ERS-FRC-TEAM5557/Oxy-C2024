package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.RobotContainer;
import frc.robot.RobotStateEstimator;
import frc.robot.subsystems.leds.Leds;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.util.DriveMotionPlanner;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import frc.robot.util.Util;

public class TeleopDrive extends Command {
    private final Swerve swerve;
    private final Translation2d passTarget = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
            .interpolate(FieldConstants.ampCenter, 0.5);

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;
    private final DoubleSupplier mAimbotXSupplier;
    private final DoubleSupplier mAimbotYSupplier;
    private final BooleanSupplier mAutoaimSupplier;
    private final BooleanSupplier mWantsAmpSnapSupplier;
    private final BooleanSupplier mWantsClimbSnapSupplier;
    private final BooleanSupplier mWantsSnapSupplier;

    private LoggedTunableNumber headingPadding = new LoggedTunableNumber("Aiming/HeadingPaddingDeg", 1.0);
    private boolean atHeadingGoal = false;

    public TeleopDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier aimbotXSupplier, DoubleSupplier aimbotYSupplier,
            BooleanSupplier autoAimSupplier, BooleanSupplier wantsAmpSnapSupplier,
            BooleanSupplier wantsClimbSnapSupplier, BooleanSupplier wantsSnapSupplier) {
        this.swerve = RobotContainer.mSwerve;
        this.mTranslationXSupplier = translationXSupplier;
        this.mTranslationYSupplier = translationYSupplier;
        this.mRotationSupplier = rotationSupplier;
        this.mAimbotXSupplier = aimbotXSupplier;
        this.mAimbotYSupplier = aimbotYSupplier;
        this.mAutoaimSupplier = autoAimSupplier;
        this.mWantsAmpSnapSupplier = wantsAmpSnapSupplier;
        this.mWantsClimbSnapSupplier = wantsClimbSnapSupplier;
        this.mWantsSnapSupplier = wantsSnapSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var limit = swerve.getKinematicLimits();
        Rotation2d driveRotation = RobotStateEstimator.getInstance().getEstimatedPose().getRotation();

        double rotationalVelocity = mRotationSupplier.getAsDouble() * limit.maxLinearVelocity();
        double xVelocity = mTranslationXSupplier.getAsDouble() * limit.maxLinearVelocity();
        double yVelocity = mTranslationYSupplier.getAsDouble() * limit.maxLinearVelocity();

        double[] rightJoyPolarCoordinate = Util.toPolarCoordinate(mAimbotYSupplier.getAsDouble(),
                mAimbotXSupplier.getAsDouble());
        double r = Util.scaledDeadband(rightJoyPolarCoordinate[0], 1.0, 0.15);
        Rotation2d theta = Rotation2d.fromRadians(rightJoyPolarCoordinate[1]);

        if (mWantsSnapSupplier.getAsBoolean()) {
            var mod = theta.getDegrees() / 45;
            theta = Rotation2d.fromDegrees(Math.round(mod) * 45);
        }

        if (AllianceFlipUtil.shouldFlip()) {
            theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        boolean wantsAutoAim = mAutoaimSupplier.getAsBoolean();
        boolean wantsAmpSnap = mWantsAmpSnapSupplier.getAsBoolean();
        boolean wantsClimbSnap = mWantsClimbSnapSupplier.getAsBoolean();
        if (r > 0.05 || wantsAutoAim || wantsAmpSnap || wantsClimbSnap) {
            if (wantsAutoAim) {
                Leds.getInstance().autoDrive = false;
                if (false) {
                    theta = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-30.0));
                } else if (AllianceFlipUtil
                        .apply(RobotStateEstimator.getInstance().getEstimatedPose().getX()) < FieldConstants.wingX) {
                    theta = RobotStateEstimator.getInstance().getAimingParameters().driveHeading();
                } else {
                    theta = AllianceFlipUtil.apply(passTarget)
                            .minus(RobotStateEstimator.getInstance().getEstimatedPose().getTranslation()).getAngle()
                            .plus(Rotation2d.fromDegrees(180.0));
                }
            } else if (wantsAmpSnap) {
                Leds.getInstance().autoDrive = false;
                theta = Rotation2d.fromDegrees(270.0);
            } else if (wantsClimbSnap) {
                Leds.getInstance().autoDrive = false;
                theta = RobotContainer.mClimbChooser.get().getPose().getRotation();

                if (AllianceFlipUtil.shouldFlip())
                    theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
            }

            rotationalVelocity = DriveMotionPlanner.calculateSnap(theta);
            atHeadingGoal = Util.epsilonEquals(theta.getDegrees(), driveRotation.getDegrees(),
                    headingPadding.getAsDouble());
        } else {
            theta = Rotation2d.fromDegrees(0.0);
            Leds.getInstance().autoDrive = false;
        }

        ChassisSpeeds velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                yVelocity,
                Util.clamp(rotationalVelocity, -limit.maxAngularVelocity(), limit.maxAngularVelocity()),
                driveRotation);

        swerve.driveOpenLoop(velocity);

        Logger.recordOutput("RobotState/Aiming/HeadingGoal", theta);

    }

    @Override
    public void end(boolean interrupted) {
        swerve.stop();
    }

    public boolean atHeadingGoal() {
        return atHeadingGoal;
    }

}

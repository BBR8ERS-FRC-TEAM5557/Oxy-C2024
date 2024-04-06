package frc.robot.subsystems.swerve.commands;

import static frc.robot.subsystems.swerve.SwerveConstants.kRotationkP;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
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
    private static final double TURNING_DEADBAND = 2.0;

    private final Swerve swerve;
    private final Translation2d passTarget = FieldConstants.Speaker.centerSpeakerOpening.toTranslation2d()
            .interpolate(FieldConstants.ampCenter, 0.5);

    private final DoubleSupplier mTranslationXSupplier;
    private final DoubleSupplier mTranslationYSupplier;
    private final DoubleSupplier mRotationSupplier;
    private final DoubleSupplier mAimbotXSupplier;
    private final DoubleSupplier mAimbotYSupplier;

    private final BooleanSupplier mAutoaimSupplier;
    private final BooleanSupplier mCustomSnapSupplier;
    private final BooleanSupplier mWantsAmpSnapSupplier;
    private final BooleanSupplier mWantsClimbSnapSupplier;
    private final BooleanSupplier mWantsSnapSupplier;

    private LoggedTunableNumber headingPadding = new LoggedTunableNumber("Aiming/HeadingPaddingDeg", 1.0);
    private boolean atHeadingGoal = false;

    private Rotation2d targetHeading;
    private double previousT;
    private double offT;

    public TeleopDrive(DoubleSupplier translationXSupplier, DoubleSupplier translationYSupplier,
            DoubleSupplier rotationSupplier, DoubleSupplier aimbotXSupplier, DoubleSupplier aimbotYSupplier,
            BooleanSupplier autoAimSupplier, BooleanSupplier customSnapSupplier, BooleanSupplier wantsAmpSnapSupplier,
            BooleanSupplier wantsClimbSnapSupplier, BooleanSupplier wantsSnapSupplier) {
        this.swerve = RobotContainer.mSwerve;
        this.mTranslationXSupplier = translationXSupplier;
        this.mTranslationYSupplier = translationYSupplier;
        this.mRotationSupplier = rotationSupplier;
        this.mAimbotXSupplier = aimbotXSupplier;
        this.mAimbotYSupplier = aimbotYSupplier;
        this.mAutoaimSupplier = autoAimSupplier;
        this.mCustomSnapSupplier = customSnapSupplier;
        this.mWantsAmpSnapSupplier = wantsAmpSnapSupplier;
        this.mWantsClimbSnapSupplier = wantsClimbSnapSupplier;
        this.mWantsSnapSupplier = wantsSnapSupplier;

        addRequirements(swerve);
    }

    @Override
    public void execute() {
        var limit = swerve.getKinematicLimits();
        Rotation2d driveRotation = RobotStateEstimator.getInstance().getEstimatedPose().getRotation();

        double rotationalVelocity = mRotationSupplier.getAsDouble() * limit.maxAngularVelocity();
        double xVelocity = mTranslationXSupplier.getAsDouble() * limit.maxLinearVelocity();
        double yVelocity = mTranslationYSupplier.getAsDouble() * limit.maxLinearVelocity();

        double[] rightJoyPolarCoordinate = Util.toPolarCoordinate(mAimbotYSupplier.getAsDouble(),
                mAimbotXSupplier.getAsDouble());
        double r = Util.scaledDeadband(rightJoyPolarCoordinate[0], 1.0, 0.15);
        Rotation2d theta = Rotation2d.fromRadians(rightJoyPolarCoordinate[1]);

        if (mWantsSnapSupplier.getAsBoolean()) {
            var mod = theta.getDegrees() / 90;
            theta = Rotation2d.fromDegrees(Math.round(mod) * 90.0);
        }

        if (AllianceFlipUtil.shouldFlip()) {
            theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
            xVelocity = -xVelocity;
            yVelocity = -yVelocity;
        }

        // r = 0.0; //replaced for liam to lucas transition

        boolean wantsAutoAim = mAutoaimSupplier.getAsBoolean();
        boolean wantsAmpSnap = mWantsAmpSnapSupplier.getAsBoolean();
        boolean wantsClimbSnap = mWantsClimbSnapSupplier.getAsBoolean();
        boolean wantsCustomSnap = mCustomSnapSupplier.getAsBoolean();
        if (r > 0.05 || wantsAutoAim || wantsAmpSnap || wantsClimbSnap || wantsCustomSnap) {
            if (wantsAutoAim) {
                if (false) {
                    theta = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-25.0));
                } else if (AllianceFlipUtil
                        .apply(RobotStateEstimator.getInstance().getEstimatedPose().getX()) < FieldConstants.wingX) {
                    theta = RobotStateEstimator.getInstance().getAimingParameters().driveHeading();
                } else {
                    theta = AllianceFlipUtil.apply(passTarget)
                            .minus(RobotStateEstimator.getInstance().getEstimatedPose().getTranslation()).getAngle()
                            .plus(Rotation2d.fromDegrees(180.0));
                }
            } else if (wantsAmpSnap) {
                theta = Rotation2d.fromDegrees(270.0);
            } else if (wantsClimbSnap) {
                theta = RobotContainer.mClimbChooser.get().getPose().getRotation();
                if (AllianceFlipUtil.shouldFlip()) {
                    theta = theta.rotateBy(Rotation2d.fromDegrees(180.0));
                }
            } else if (wantsCustomSnap) {
                theta = AllianceFlipUtil.apply(Rotation2d.fromDegrees(-25.0));
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

        // correctHeading(velocity, driveRotation); //potential failure point idek??

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

    /**
     * Optimizes the chassis speed that is put into the kinematics object to allow
     * the robot to hold its heading
     * when no angular velocity is input.
     * The robot will therefore correct itself when it turns without telling it to
     * do so.
     *
     * @param desiredSpeed desired chassis speed that is input by the controller
     * @return corrected {@code ChassisSpeeds} which takes into account that the
     *         robot needs to have the same heading
     *         when no rotational speed is input
     */
    private ChassisSpeeds correctHeading(ChassisSpeeds desiredSpeed, Rotation2d driveRotation) {

        // Determine time interval
        double currentT = Timer.getFPGATimestamp();
        double dt = currentT - previousT;

        // Get desired rotational speed in radians per second and absolute translational
        // speed in m/s
        double vr = desiredSpeed.omegaRadiansPerSecond;
        double v = Math.hypot(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond);

        if (vr > 0.01 || vr < -0.01) {
            offT = currentT;
            targetHeading = driveRotation;
            return desiredSpeed;
        }
        if (currentT - offT < 0.5) {
            targetHeading = driveRotation;
            return desiredSpeed;
        }
        if (v < 0.1) {
            targetHeading = driveRotation;
            return desiredSpeed;
        }

        // Determine target and current heading
        // Update target heading to account for any requested vr less than 0.01
        targetHeading = targetHeading.plus(new Rotation2d(vr * dt));

        // Calculate the change in heading that is needed to achieve the target
        Rotation2d deltaHeading = targetHeading.minus(driveRotation);

        if (Math.abs(deltaHeading.getDegrees()) < TURNING_DEADBAND) {
            return desiredSpeed;
        }
        double correctedVr = deltaHeading.getRadians() / dt * kRotationkP;

        previousT = currentT;

        return new ChassisSpeeds(desiredSpeed.vxMetersPerSecond, desiredSpeed.vyMetersPerSecond, correctedVr);
    }

}

package frc.robot.auto;

import java.util.HashMap;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

//import com.pathplanner.lib.commands.PPSwerveControllerCommand;
//import com.pathplanner.lib.commands.FollowPathWith;
//import com.pathplanner.lib.server.PathPlannerServer;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.intake.Intake;
//import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.FeedForwardCharacterization;
import frc.robot.util.RobotStateEstimator;

public class AutoRoutineManager {
    private final LoggedDashboardChooser<Command> mChooser;
    private final HashMap<String, Command> mEventMap;
    private final HashMap<String, PathPlannerTrajectory> mTrajectoryMap;

    private final Swerve swerve;
    private final Intake intake;
    private final Feeder feeder;
    private final Flywheels flywheels;
    private final Arm arm;

    public AutoRoutineManager(Swerve swerve, Intake intake, Feeder feeder, Flywheels flywheels, Arm arm) {
        System.out.println("[Init] Creating Auto Routine Manager");
        mChooser = new LoggedDashboardChooser<Command>("Driver/AutonomousChooser");
        mEventMap = new HashMap<>();
        mTrajectoryMap = new HashMap<>();

        this.swerve = swerve;
        this.intake = intake;
        this.feeder = feeder;
        this.flywheels = flywheels;
        this.arm = arm;

        generateAutoChoices();
    }

    private void generateAutoChoices() {
        mChooser.addDefaultOption("Do Nothing", null);

        

        // Set up feedforward characterization
        mChooser.addOption(
                "Drive FF Characterization",
                new FeedForwardCharacterization(
                        swerve, swerve::runCharacterizationVolts, swerve::getCharacterizationVelocity)
                        .finallyDo(swerve::stop));

        mChooser.addOption(
                "Flywheels FF Characterization",
                new FeedForwardCharacterization(
                        flywheels, flywheels::runCharacterizationVolts, flywheels::getCharacterizationVelocity)
                        .finallyDo(flywheels::stop));

    }

    public Command getAutoCommand() {
        return mChooser.get();
    }

    private void setPose(Pose2d pose) {
        RobotStateEstimator.getInstance().setPose(pose);
    }

}

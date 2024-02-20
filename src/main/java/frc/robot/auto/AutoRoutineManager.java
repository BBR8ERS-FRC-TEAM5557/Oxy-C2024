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
//import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.util.RobotStateEstimator;

public class AutoRoutineManager {
    private final LoggedDashboardChooser<Command> m_chooser;
    private final HashMap<String, Command> m_eventMap;
    private final HashMap<String, PathPlannerTrajectory> m_trajectoryMap;

    private final Swerve swerve;
    private final Arm arm;
    // private final Feeder feeder;
    // private final Intake intake;
    //private final Flywheels flywheels;


    public AutoRoutineManager(Swerve swerve, Arm arm) {
        System.out.println("[Init] Creating Auto Routine Manager");
        m_chooser = new LoggedDashboardChooser<Command>("AutonomousChooser");
        m_eventMap = new HashMap<>();
        m_trajectoryMap = new HashMap<>();

        this.swerve = swerve;
        this.arm = arm;
        
        generateAutoChoices();

    }

    private void generateAutoChoices() {
        m_chooser.addDefaultOption("Do Nothing", null);

        
    }

        public Command getAutoCommand() {
        return m_chooser.get();
    }

    private void setPose(Pose2d pose) {
        RobotStateEstimator.getInstance().setPose(pose);
    }

}

package frc.robot.subsystems.superstructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.arm.Arm;
import frc.robot.subsystems.shooter.roller.Roller;

public class Superstructure {
    
    private static final Intake intake = RobotContainer.m_intake;
    private static final Arm arm = RobotContainer.m_arm;
    private static final Roller roller = RobotContainer.m_roller;

    public static SuperstructureGoal currentGoal = SuperstructureGoal.STOW;

    public static SuperstructureGoal getCurrentGoal() {
            return currentGoal;
    }

    public static void setCurrentGoal(SuperstructureGoal cur) {
            currentGoal = cur;
    }

    public static Command setSuperstructureGoal(SuperstructureGoal goal) {
            return Commands.sequence(
                            new InstantCommand(() -> setCurrentGoal(goal)));
    }

    public static Command setStow() {
            return setSuperstructureGoal(SuperstructureGoal.STOW);
    }

    public static Command scoreAmp(){
            return setSuperstructureGoal(SuperstructureGoal.SCORE_AMP);
    }

    /**
    public static Command intakeNote(){
        return Commands new InstantCommand(() -> Commands.deadline(roller.intakeNoteCommand(),
                setSuperstructureGoal(SuperstructureGoal.GROUND_INTAKE)));
    } */

    public static enum SuperstructureGoal {
            STOW(260.0),

            SCORE_STANDBY(200.0),
            
            STATION_INTAKE(75.0), SCORE_AMP(50.0),
            
            GROUND_INTAKE(260.0), 
            
            SCORE_SPEAKER(110.0);

            public double arm; // degrees

            private SuperstructureGoal(double arm) {
                    this.arm = arm;
            }

            // Default Constructor
            private SuperstructureGoal() {
                    this(0);
            }

    }

}

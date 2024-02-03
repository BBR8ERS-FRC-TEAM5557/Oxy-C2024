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

public class Superstructure {
    
    private static final Intake intake = RobotContainer.m_intake;
   

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

    public static enum SuperstructureGoal {
            STOW(260.0),

            SCORE_STANDBY(200.0),
            
            STATION_INTAKE(75.0);

            public double wrist; // degrees

            private SuperstructureGoal(double wrist) {
                    this.wrist = wrist;
            }

            // Default Constructor
            private SuperstructureGoal() {
                    this(0);
            }

    }



}

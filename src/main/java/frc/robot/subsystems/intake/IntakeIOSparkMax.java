package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;

import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;

public class IntakeIOSparkMax implements IntakeIO{

        private CANSparkMax motor;

        public IntakeIOSparkMax(){

            System.out.println("[Init] Creating IntakeIOSparkMax");
            //motor = SparkMaxFactory.createNEO(kIntakeMotorConfiguration);
            //BurnManager.burnFlash(motor);
            
        } 

        //public void updateInputs

}

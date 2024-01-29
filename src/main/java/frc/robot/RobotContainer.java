// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoRoutineManager;
import frc.robot.auto.SystemsCheckManager;
import frc.robot.subsystems.climb.*;
import frc.robot.subsystems.deflector.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.swerve.*;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();
  }

  private static final XboxController m_driver = new XboxController(0);
  private static final XboxController m_operator = new XboxController(1);

  public static Swerve m_swerve;
  public static Climb m_climb;
  public static Deflector m_deflector;

  public static AutoRoutineManager m_autoManager;
  public static SystemsCheckManager m_systemCheckManager;

  //m_autoManager = new AutoRoutineManager(m_swerve);

    private void configureBindings() {}


    public Command getAutonomousCommand() {
      return m_autoManager.getAutoCommand();
}

    public Command getSubsystemCheckCommand() {
      return m_systemCheckManager.getCheckCommand();
}

    public double getForwardInput() {
      return -square(deadband(m_driver.getLeftY(), 0.15));
}

    public double getStrafeInput() {
    return -square(deadband(m_driver.getLeftX(), 0.15));
}

    public double getRotationInput() {
    return -square(deadband(m_driver.getRightX(), 0.15));
}

    private static double deadband(double value, double tolerance) {
      if (Math.abs(value) < tolerance)
          return 0.0;

          return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
    }

    public static double square(double value) {
        return Math.copySign(value * value, value);
}
}

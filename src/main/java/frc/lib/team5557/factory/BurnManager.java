package frc.lib.team5557.factory;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

public class BurnManager {
  private static boolean shouldBurn = true;

  public static void restoreFactoryDefaults(CANSparkMax sparkMax) {
    if (shouldBurn) {
      sparkMax.restoreFactoryDefaults();
      Timer.delay(0.2);
    }
  }

  public static void burnFlash(CANSparkMax sparkMax) {
    if (shouldBurn) {
      sparkMax.burnFlash();
      Timer.delay(0.2);
    }
  }
/**
  public static void burnFlash(TalonFX talon) {
    if (shouldBurn) {
      talon.getConfigurator().apply(null);
    }
    Timer.delay(.2);

  } */

  public static boolean shouldBurn() {
    return shouldBurn;
  }
}

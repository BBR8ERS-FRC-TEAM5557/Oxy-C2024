package frc.lib.team5557.factory;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.team5557.util.CANDeviceFinder;
import frc.lib.team5557.util.CANDeviceId;
import frc.lib.team5557.util.CANDeviceType;

public class TalonFactory {
    public static final int configCANTimeout = 500;
    public static final int configCount = 4;

    private static final CANDeviceFinder can = new CANDeviceFinder();

    public static class Configuration {
        public String label;
        public CANDeviceId canID;

        public boolean setInverted = false;
        public NeutralModeValue neutralMode = NeutralModeValue.Brake;

        public PIDConfiguration pid = null;

        public SoftwareLimitSwitchConfigs limits = null;

        public double supplyCurrentLimit = Double.NaN;
        public double statorCurrentLimit = Double.NaN;

        public double voltageCompensation = Double.NaN;
        public double openLoopRampRate = 0.0;
        public double closedLoopRampRate = 0.0;
    }

    public static class PIDConfiguration {
        public double kP = 0.0;
        public double kI = 0.0;
        public double kD = 0.0;

        public double kS = 0.0;
        public double kG = 0.0;
        public double kV = 0.0;
        public double kA = 0.0;
    }

    public static class SoftLimitsConfiguration {
        public double upperLimit = Double.NaN;
        public double lowerLimit = Double.NaN;
    }

    public static TalonFX createTalon(int id, Configuration config) {
        config.canID = new CANDeviceId(CANDeviceType.TALON_PHOENIX6, id);
        return createTalon(config);
    }

    public static TalonFX createTalon(Configuration config) {
        can.isDevicePresent(config.canID.getDeviceType(), config.canID.getDeviceNumber(), config.label);

        TalonFX talon = new TalonFX(config.canID.getDeviceNumber());

        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

        //general
        talonFXConfiguration.MotorOutput.Inverted = config.setInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        talonFXConfiguration.MotorOutput.NeutralMode = config.neutralMode;

        //PID
        if (config.pid != null) {
            talonFXConfiguration.Slot0.kP = config.pid.kP;
            talonFXConfiguration.Slot0.kI = config.pid.kI;
            talonFXConfiguration.Slot0.kD = config.pid.kD;
        }

        //Software Limits
        if (config.limits != null) {
            talonFXConfiguration.SoftwareLimitSwitch = config.limits;
        }

        //Current Limits
        if (!Double.isNaN(config.supplyCurrentLimit)) {
            talonFXConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            talonFXConfiguration.CurrentLimits.SupplyCurrentThreshold = 100;
            talonFXConfiguration.CurrentLimits.SupplyTimeThreshold = 0.1;
            talonFXConfiguration.CurrentLimits.SupplyCurrentLimit = config.supplyCurrentLimit;
        }
        if (!Double.isNaN(config.statorCurrentLimit)) {
            talonFXConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
            talonFXConfiguration.CurrentLimits.StatorCurrentLimit = config.statorCurrentLimit;
        }

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 4; ++i) {
            status = talon.getConfigurator().apply(talonFXConfiguration);
            if (status.isOK())
                break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        return talon;
    }
}

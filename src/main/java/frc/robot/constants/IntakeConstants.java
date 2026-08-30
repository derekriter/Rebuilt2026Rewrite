package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public final class IntakeConstants {
    public static final class RollerConstants {
        public static final int canID = 14;
        public static final int channelID = 14;

        public static final TalonFXConfiguration motorConfig;
        public static final Temperature tempWarnThreshold = MotorConstants.falcon500TempWarnThreshold;
        public static final Temperature thermalShutdownThreshold = MotorConstants.falcon500ThermalShutdownThreshold;

        public static final Voltage intakeVoltage = Volts.of(12);
        public static final Voltage shootVoltage = Volts.of(12);
        public static final Voltage reverseVoltage = Volts.of(-4);

        static {
            motorConfig = new TalonFXConfiguration();

            motorConfig.CurrentLimits.StatorCurrentLimit = 60;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfig.Voltage.PeakForwardVoltage = 12;
            motorConfig.Voltage.PeakReverseVoltage = 12;

            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
    }

    public static final class LeftDeployerConstants {
        public static final int port = 1;
        public static final int channelID = 0; // TODO: left deployer channel id

        public static final double deployedPosition = 1;
    }

    public static final class RightDeployerConstants {
        public static final int port = 3;
        public static final int channelID = 0; // TODO: right deployer channel id

        public static final double deployedPosition = 1;
    }

    private IntakeConstants() {}
}

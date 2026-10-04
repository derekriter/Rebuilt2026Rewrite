package frc.robot.constants;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;

public final class IndexerConstants {
    public static final class SpindexerConstants {
        public static final int canID = 17;
        public static final int channelID = 17;

        public static final TalonFXConfiguration motorConfig;

        public static final Voltage shootVoltage = Volts.of(12);
        public static final Voltage slowReverseVoltage = Volts.of(-1.5 * (16.0 / 27));
        public static final Voltage fastReverseVoltage = Volts.of(-6 * (16.0 / 27));

        static {
            motorConfig = new TalonFXConfiguration();

            motorConfig.CurrentLimits.StatorCurrentLimit = 60;
            motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
            motorConfig.CurrentLimits.SupplyCurrentLimit = 40;
            motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

            motorConfig.Voltage.PeakForwardVoltage = 12;
            motorConfig.Voltage.PeakReverseVoltage = -12;

            motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
            motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
    }

    public static final class ExchangeConstants {
        public static final int canID = 16;
        public static final int channelID = 16;

        public static final SparkMaxConfig motorConfig;

        public static final Voltage shootVoltage = Volts.of(12);

        static {
            motorConfig = new SparkMaxConfig();

            motorConfig.smartCurrentLimit(30);
            motorConfig.voltageCompensation(12);
            motorConfig.idleMode(IdleMode.kCoast);
            motorConfig.inverted(false);
            motorConfig.closedLoop.outputRange(-1, 1);

            motorConfig.signals.primaryEncoderVelocityAlwaysOn(true).primaryEncoderVelocityPeriodMs(20); // status 2
            motorConfig.signals.motorTemperaturePeriodMs(20); // status 0
            motorConfig.signals.appliedOutputPeriodMs(20); // status 0
            motorConfig.signals.busVoltagePeriodMs(20); // status 0
            motorConfig.signals.outputCurrentPeriodMs(20); // status 0
            motorConfig.signals.faultsAlwaysOn(true).faultsPeriodMs(20); // status 1
        }
    }

    private IndexerConstants() {}
}

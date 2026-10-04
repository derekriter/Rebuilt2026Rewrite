package frc.robot.subsystems.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface IIndexerIO {

    public static final IIndexerIO blank = new IIndexerIO() {
        @Override
        public void updateInputs(IndexerIOInputs inputs) {}

        @Override
        public void setSpindexerVoltage(double voltage_V) {}

        @Override
        public void setExchangeVoltage(double voltage_V) {}
    };

    public static class IndexerIOInputs implements LoggableInputs {
        public boolean spindexerConnected = false;
        public double spindexerVel_RPM = Double.NaN;
        public double spindexerTemp_C = Double.NaN;
        public double spindexerStatorVoltage_V = Double.NaN;
        public double spindexerStatorCurrent_A = Double.NaN;
        public double spindexerSupplyCurrent_A = Double.NaN;
        public boolean spindexerThermalShutdown = false;

        public boolean exchangeConnected = false;
        public double exchangeVel_RPM = Double.NaN;
        public double exchangeTemp_C = Double.NaN;
        public double exchangeAppliedOut_perc = Double.NaN;
        public double exchangeStatorVoltage_V = Double.NaN;
        public double exchangeStatorCurrent_A = Double.NaN;
        public boolean exchangeThermalShutdown = false;

        @Override
        public void toLog(LogTable table) {
            table.put("spindexerConnected", spindexerConnected);
            table.put("spindexerVel", spindexerVel_RPM, RPM.name());
            table.put("spindexerTemp", spindexerTemp_C, Celsius.name());
            table.put("spindexerStatorVoltage", spindexerStatorVoltage_V, Volts.name());
            table.put("spindexerStatorCurrent", spindexerStatorCurrent_A, Amps.name());
            table.put("spindexerSupplyCurrent", spindexerSupplyCurrent_A, Amps.name());
            table.put("spindexerThermalShutdown", spindexerThermalShutdown);

            table.put("exchangeConnected", exchangeConnected);
            table.put("exchangeVel", exchangeVel_RPM, RPM.name());
            table.put("exchangeTemp", exchangeTemp_C, Celsius.name());
            table.put("exchangeStatorVoltage", exchangeStatorVoltage_V, Volts.name());
            table.put("exchangeStatorCurrent", exchangeStatorCurrent_A, Amps.name());
            table.put("exchangeThermalShutdown", exchangeThermalShutdown);
        }

        @Override
        public void fromLog(LogTable table) {
            spindexerConnected = table.get("connected", spindexerConnected);
            spindexerVel_RPM = table.get("vel", spindexerVel_RPM);
            spindexerTemp_C = table.get("temp", spindexerTemp_C);
            spindexerStatorVoltage_V = table.get("statorVoltage", spindexerStatorVoltage_V);
            spindexerStatorCurrent_A = table.get("statorCurrent", spindexerStatorCurrent_A);
            spindexerSupplyCurrent_A = table.get("supplyCurrent", spindexerSupplyCurrent_A);
            spindexerThermalShutdown = table.get("thermalShutdown", spindexerThermalShutdown);

            exchangeConnected = table.get("exchangeConnected", exchangeConnected);
            exchangeVel_RPM = table.get("exchangeVel", exchangeVel_RPM);
            exchangeTemp_C = table.get("exchangeTemp", exchangeTemp_C);
            exchangeStatorVoltage_V = table.get("exchangeStatorVoltage", exchangeStatorVoltage_V);
            exchangeStatorCurrent_A = table.get("exchangeStatorCurrent", exchangeStatorCurrent_A);
            exchangeThermalShutdown = table.get("exchangeThermalShutdown", exchangeThermalShutdown);
        }
    }

    public void updateInputs(IndexerIOInputs inputs);

    public void setSpindexerVoltage(double voltage_V);

    public void setExchangeVoltage(double voltage_V);
}

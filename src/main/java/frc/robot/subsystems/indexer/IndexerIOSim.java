package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;

public class IndexerIOSim implements IIndexerIO {

    private double spindexerVoltage_V = 0;
    private double exchangeVoltage_V = 0;

    public IndexerIOSim() {}

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        inputs.spindexerConnected = true;
        inputs.spindexerVel_RPM = 5800 * spindexerVoltage_V / 12.0;
        inputs.spindexerTemp_C = 20;
        inputs.spindexerStatorVoltage_V = spindexerVoltage_V;
        inputs.spindexerStatorCurrent_A = 0;
        inputs.spindexerSupplyCurrent_A = 0;
        inputs.spindexerThermalShutdown = false;

        inputs.exchangeConnected = true;
        inputs.exchangeVel_RPM = 5700 * exchangeVoltage_V / 12.0;
        inputs.exchangeTemp_C = 20;
        inputs.exchangeAppliedOut_perc = exchangeVoltage_V / 12.0;
        inputs.exchangeStatorVoltage_V = exchangeVoltage_V;
        inputs.exchangeStatorCurrent_A = 0;
        inputs.exchangeThermalShutdown = false;
    }

    @Override
    public void setSpindexerVoltage(double voltage_V) {
        spindexerVoltage_V = MathUtil.clamp(voltage_V, -12, 12);
    }

    @Override
    public void setExchangeVoltage(double voltage_V) {
        exchangeVoltage_V = MathUtil.clamp(voltage_V, -12, 12);
    }
}

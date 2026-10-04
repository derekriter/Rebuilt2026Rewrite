package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.IndexerConstants.ExchangeConstants;
import frc.robot.constants.IndexerConstants.SpindexerConstants;
import frc.robot.util.MotorUtils;

public class IndexerIOReal implements IIndexerIO {

    private final TalonFX spindexer;
    private final StatusSignal<AngularVelocity> spindexerVel_rps;
    private final StatusSignal<Temperature> spindexerTemp_C;
    private final StatusSignal<Voltage> spindexerStatorVoltage_V;
    private final StatusSignal<Current> spindexerStatorCurrent_A;
    private final StatusSignal<Current> spindexerSupplyCurrent_A;
    private final StatusSignal<Boolean> spindexerThermalShutdown;
    private final Debouncer spindexerConnectedDebouncer = new Debouncer(0.5, DebounceType.kFalling);

    private final SparkMax exchange;

    public IndexerIOReal() {
        spindexer = new TalonFX(SpindexerConstants.canID);
        MotorUtils.safeApplyConfig(
                spindexer,
                "spindexer",
                SpindexerConstants.canID,
                SpindexerConstants.channelID,
                SpindexerConstants.motorConfig);

        spindexerVel_rps = spindexer.getVelocity();
        spindexerTemp_C = spindexer.getDeviceTemp();
        spindexerStatorVoltage_V = spindexer.getMotorVoltage();
        spindexerStatorCurrent_A = spindexer.getStatorCurrent();
        spindexerSupplyCurrent_A = spindexer.getSupplyCurrent();
        spindexerThermalShutdown = spindexer.getFault_DeviceTemp();
        BaseStatusSignal.setUpdateFrequencyForAll(
                50,
                spindexerVel_rps,
                spindexerTemp_C,
                spindexerStatorVoltage_V,
                spindexerStatorCurrent_A,
                spindexerSupplyCurrent_A,
                spindexerThermalShutdown);
        spindexer.optimizeBusUtilization();

        exchange = new SparkMax(ExchangeConstants.canID, MotorType.kBrushless);
        MotorUtils.safeApplyConfig(
                exchange,
                "exchange",
                ExchangeConstants.canID,
                ExchangeConstants.channelID,
                ExchangeConstants.motorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        StatusCode spindexerStatus = BaseStatusSignal.refreshAll(
                spindexerVel_rps,
                spindexerTemp_C,
                spindexerStatorVoltage_V,
                spindexerStatorCurrent_A,
                spindexerSupplyCurrent_A,
                spindexerThermalShutdown);

        inputs.spindexerConnected = spindexerConnectedDebouncer.calculate(spindexerStatus.isOK());
        inputs.spindexerVel_RPM = spindexerVel_rps.getValueAsDouble() * 60;
        inputs.spindexerTemp_C = spindexerTemp_C.getValueAsDouble();
        inputs.spindexerStatorVoltage_V = spindexerStatorVoltage_V.getValueAsDouble();
        inputs.spindexerStatorCurrent_A = spindexerStatorCurrent_A.getValueAsDouble();
        inputs.spindexerSupplyCurrent_A = spindexerSupplyCurrent_A.getValueAsDouble();
        inputs.spindexerThermalShutdown = spindexerThermalShutdown.getValue();

        inputs.exchangeConnected = MotorUtils.isSparkConnected(exchange);
        if (inputs.exchangeConnected) {
            inputs.exchangeVel_RPM = exchange.getEncoder().getVelocity();
            inputs.exchangeTemp_C = exchange.getMotorTemperature();
            inputs.exchangeAppliedOut_perc = exchange.getAppliedOutput();
            inputs.exchangeStatorVoltage_V = exchange.getBusVoltage() * inputs.exchangeAppliedOut_perc;
            inputs.exchangeStatorCurrent_A = exchange.getOutputCurrent();
            inputs.exchangeThermalShutdown = MotorUtils.isSparkThermalShutdown(exchange);
        }
    }

    @Override
    public void setSpindexerVoltage(double voltage_V) {
        spindexer.setVoltage(voltage_V);
    }

    @Override
    public void setExchangeVoltage(double voltage_V) {
        exchange.setVoltage(voltage_V);
    }
}

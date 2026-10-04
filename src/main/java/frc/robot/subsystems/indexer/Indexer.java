package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.IndexerConstants.ExchangeConstants;
import frc.robot.constants.IndexerConstants.SpindexerConstants;
import frc.robot.constants.Overrides;
import frc.robot.subsystems.indexer.IIndexerIO.IndexerIOInputs;
import frc.robot.util.AlertUtils;
import frc.robot.util.BlankValues;
import frc.robot.util.Console;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

    private final IIndexerIO io_nl;
    private IndexerIOInputs inputs = new IndexerIOInputs();

    private final Alert spindexerCANAlert = AlertUtils.makeCANFailureAlert("spindexer"),
            spindexerBreakerAlert = AlertUtils.makeBreakerTripAlert("spindexer"),
            spindexerThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert("spindexer"),
            exchangeCANAlert = AlertUtils.makeCANFailureAlert("exchange"),
            exchangeBreakerAlert = AlertUtils.makeBreakerTripAlert("exchange"),
            exchangeThermalShutdownAlert = AlertUtils.makeThermalShutdownAlert("exchange");

    private boolean spindexerConnectedLast = false;
    private boolean spindexerBreaker = false;
    private boolean spindexerBreakerLast = false;
    private boolean spindexerThermalShutdownLast = false;

    private boolean exchangeConnectedLast = false;
    private boolean exchangeBreaker = false;
    private boolean exchangeBreakerLast = false;
    private boolean exchangeThermalShutdownLast = false;

    public Indexer(IIndexerIO _io_nl) {
        io_nl = _io_nl;
        if (io_nl == null) {
            Logger.recordOutput("CAN/spindexer_" + SpindexerConstants.canID, false);
            Logger.recordOutput("CAN/exchange" + ExchangeConstants.canID, false);
            AlertUtils.makeSystemDisabledAlert("indexer").set(true);
        } else {
            if (Overrides.disableIndexerSafety) {
                AlertUtils.makeSafetyDisabledAlert("indexer").set(true);
            }
        }
    }

    @Override
    public void periodic() {
        if (io_nl != null) {
            io_nl.updateInputs(inputs);
            Logger.processInputs("IndexerInputs", inputs);

            // spindexer
            {
                spindexerBreaker = RobotContainer.instance().pdh.isBreakerTripped(SpindexerConstants.channelID);

                Logger.recordOutput("CAN/spindexer_" + SpindexerConstants.canID, inputs.spindexerConnected);
                spindexerCANAlert.set(!inputs.spindexerConnected && !spindexerBreaker);
                if (inputs.spindexerConnected != spindexerConnectedLast) {
                    if (inputs.spindexerConnected) {
                        Console.reportCANConnect("spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    } else {
                        Console.reportCANDisconnect(
                                "spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    }
                }
                Logger.recordOutput("Indexer/Spindexer/breakerTripped", spindexerBreaker);
                spindexerBreakerAlert.set(spindexerBreaker);
                if (spindexerBreaker != spindexerBreakerLast) {
                    if (spindexerBreaker) {
                        Console.reportBreakerTrip("spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    } else {
                        Console.reportBreakerReset("spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    }
                }
                spindexerThermalShutdownAlert.set(inputs.spindexerThermalShutdown);
                if (inputs.spindexerThermalShutdown != spindexerThermalShutdownLast) {
                    if (inputs.spindexerThermalShutdown) {
                        Console.reportThermalShutdownTrigger(
                                "spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    } else {
                        Console.reportThermalShutdownRelease(
                                "spindexer", SpindexerConstants.canID, SpindexerConstants.channelID);
                    }
                }

                spindexerConnectedLast = inputs.spindexerConnected;
                spindexerBreakerLast = spindexerBreaker;
                spindexerThermalShutdownLast = inputs.spindexerThermalShutdown;
            }

            // exchange
            {
                exchangeBreaker = RobotContainer.instance().pdh.isBreakerTripped(ExchangeConstants.channelID);

                Logger.recordOutput("CAN/exchange_" + ExchangeConstants.canID, inputs.exchangeConnected);
                exchangeCANAlert.set(!inputs.exchangeConnected && !exchangeBreaker);
                if (inputs.exchangeConnected != exchangeConnectedLast) {
                    if (inputs.exchangeConnected) {
                        Console.reportCANConnect("exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    } else {
                        Console.reportCANDisconnect("exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    }
                }
                Logger.recordOutput("Indexer/Spindexer/breakerTripped", exchangeBreaker);
                exchangeBreakerAlert.set(exchangeBreaker);
                if (exchangeBreaker != exchangeBreakerLast) {
                    if (exchangeBreaker) {
                        Console.reportBreakerTrip("exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    } else {
                        Console.reportBreakerReset("exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    }
                }
                exchangeThermalShutdownAlert.set(inputs.exchangeThermalShutdown);
                if (inputs.exchangeThermalShutdown != exchangeThermalShutdownLast) {
                    if (inputs.exchangeThermalShutdown) {
                        Console.reportThermalShutdownTrigger(
                                "exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    } else {
                        Console.reportThermalShutdownRelease(
                                "exchange", ExchangeConstants.canID, ExchangeConstants.channelID);
                    }
                }

                exchangeConnectedLast = inputs.exchangeConnected;
                exchangeBreakerLast = exchangeBreaker;
                exchangeThermalShutdownLast = inputs.exchangeThermalShutdown;
            }
        }

        Command currentCommand = getCurrentCommand();
        Logger.recordOutput(
                "Indexer/currentCommand", currentCommand == null ? BlankValues.string : currentCommand.getName());

        Command defaultCommand = getDefaultCommand();
        Logger.recordOutput(
                "Indexer/defaultCommand", defaultCommand == null ? BlankValues.string : defaultCommand.getName());
    }

    public void report(IndexerReport report) {
        if (Overrides.disableIndexerSafety) {
            report.isOperational = true;
        } else if (io_nl == null) {
            report.isOperational = false;
        } else {
            report.isOperational = (inputs.spindexerConnected && !spindexerBreaker && !inputs.spindexerThermalShutdown)
                    && (inputs.exchangeConnected && !exchangeBreaker && !inputs.exchangeThermalShutdown);
        }
    }

    public void setSpindexerVoltage(double voltage_V) {
        if (io_nl == null) return;

        io_nl.setSpindexerVoltage(voltage_V);
    }

    public void stopSpindexer() {
        setSpindexerVoltage(0);
    }

    public void setExchangeVoltage(double voltage_V) {
        if (io_nl == null) return;

        io_nl.setExchangeVoltage(voltage_V);
    }

    public void stopExchange() {
        setExchangeVoltage(0);
    }
}

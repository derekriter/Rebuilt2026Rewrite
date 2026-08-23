package frc.robot.pdh;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.config.Overrides;
import frc.robot.config.PDHConfig;
import frc.robot.logging.LoggingUnits;
import frc.robot.logging.Telemetry;
import frc.robot.logging.writer.BoolArrayWriter;
import frc.robot.logging.writer.BoolWriter;
import frc.robot.logging.writer.DoubleArrayWriter;
import frc.robot.logging.writer.DoubleWriter;
import frc.robot.util.AlertUtils;

public class PDH {

    private final PowerDistribution powerDistribution_nl;

    private PDHBuffer buffer = new PDHBuffer();
    private boolean connectedLast = false;

    private final Alert canAlert = AlertUtils.makeCANFailureAlert(PDHConfig.systemName);

    private final DoubleWriter voltageWriter_nl;
    private final DoubleWriter totalCurrentWriter_nl;
    private final DoubleArrayWriter currentsWriter_nl;
    private final BoolArrayWriter breakersWriter_nl;
    private final BoolWriter connWriter_nl, canWriter;

    public PDH() {
        canWriter = Telemetry.makeBoolWriter("CAN", String.format("%s_%s", PDHConfig.systemName, PDHConfig.canID));

        if (Overrides.disablePDHMonitoring) {
            powerDistribution_nl = null;
            voltageWriter_nl = null;
            totalCurrentWriter_nl = null;
            connWriter_nl = null;
            currentsWriter_nl = null;
            breakersWriter_nl = null;

            AlertUtils.makeSystemDisabledAlert("PDH monitoring").set(true);
            canWriter.set(false);
            canWriter.close();
        } else {
            powerDistribution_nl = new PowerDistribution(PDHConfig.canID, PDHConfig.type);
            buffer.breakersTripped = new boolean[powerDistribution_nl.getNumChannels()];

            voltageWriter_nl = Telemetry.makeDoubleWriter(PDHConfig.systemName, "voltage", LoggingUnits.volts);
            totalCurrentWriter_nl = Telemetry.makeDoubleWriter(PDHConfig.systemName, "totalCurrent", LoggingUnits.amps);
            connWriter_nl = Telemetry.makeBoolWriter(PDHConfig.systemName, "connected");
            currentsWriter_nl = Telemetry.makeDoubleArrayWriter(PDHConfig.systemName, "currents", LoggingUnits.amps);
            breakersWriter_nl = Telemetry.makeBoolArrayWriter(PDHConfig.systemName, "breakers");
        }
    }

    public void update() {
        if (powerDistribution_nl == null) return;

        buffer.voltage_V = powerDistribution_nl.getVoltage();
        buffer.connected = buffer.voltage_V != 0;

        if (buffer.connected) {
            buffer.totalCurrent_A = powerDistribution_nl.getTotalCurrent();
            buffer.currents_A = powerDistribution_nl.getAllCurrents();

            PowerDistributionFaults faults = powerDistribution_nl.getFaults();
            for (int i = 0; i < buffer.breakersTripped.length; i++) {
                buffer.breakersTripped[i] = faults.getBreakerFault(i);
            }
        } else {
            buffer.voltage_V = Double.NaN;
            buffer.totalCurrent_A = Double.NaN;
        }

        connWriter_nl.set(buffer.connected);
        canWriter.set(buffer.connected);
        canAlert.set(!buffer.connected);
        if (buffer.connected != connectedLast) {
            if (buffer.connected) {
                Telemetry.reportCANConnectNoChannel(PDHConfig.systemName, PDHConfig.canID);
            } else {
                Telemetry.reportCANDisconnectNoChannel(PDHConfig.systemName, PDHConfig.canID);
            }
        }
        voltageWriter_nl.set(buffer.voltage_V);
        totalCurrentWriter_nl.set(buffer.totalCurrent_A);
        currentsWriter_nl.set(buffer.connected ? buffer.currents_A : null);
        breakersWriter_nl.set(buffer.connected ? buffer.breakersTripped : null);

        connectedLast = buffer.connected;
    }

    public boolean isBreakerTripped(int channel) {
        if (powerDistribution_nl == null
                || !buffer.connected
                || channel < 0
                || channel >= buffer.breakersTripped.length) return false;

        return buffer.breakersTripped[channel];
    }
}

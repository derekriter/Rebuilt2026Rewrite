package frc.robot.pdh;

import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.PowerDistribution;
import frc.robot.config.Overrides;
import frc.robot.config.PDHConfig;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolArrayWriter;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleArrayWriter;
import frc.robot.telemetry.writer.DoubleWriter;
import frc.robot.util.AlertUtils;
import java.util.Optional;

public class PDH {

    private final Optional<PowerDistribution> powerDistribution;

    private PDHBuffer buffer = new PDHBuffer();

    private final Alert disconnAlert = AlertUtils.makeDisconnectAlert(PDHConfig.systemName, PDHConfig.canID);

    private final DoubleWriter voltageWriter;
    private final DoubleWriter totalCurrentWriter;
    private final DoubleArrayWriter currentsWriter;
    private final BoolArrayWriter breakersWriter;
    private final BoolWriter connWriter, canWriter;

    public PDH() {
        canWriter = Telemetry.makeBoolWriter("CAN", String.format("%s_%s", PDHConfig.systemName, PDHConfig.canID));

        if (Overrides.disablePDHMonitoring) {
            powerDistribution = Optional.empty();

            AlertUtils.makeSystemDisabledAlert("PDH monitoring").set(true);
            canWriter.set(false);
            canWriter.close();
        } else {
            powerDistribution = Optional.of(new PowerDistribution(PDHConfig.canID, PDHConfig.type));
            buffer.breakersTripped = new boolean[powerDistribution.get().getNumChannels()];

            voltageWriter = Telemetry.makeDoubleWriter(PDHConfig.systemName, "voltage", TelemetryUnits.volts);
            totalCurrentWriter = Telemetry.makeDoubleWriter(PDHConfig.systemName, "totalCurrent", TelemetryUnits.amps);
            connWriter = Telemetry.makeBoolWriter(PDHConfig.systemName, "connected");
            currentsWriter = Telemetry.makeDoubleArrayWriter(PDHConfig.systemName, "currents", TelemetryUnits.amps);
            breakersWriter = Telemetry.makeBoolArrayWriter(PDHConfig.systemName, "breakers");
        }
    }

    public void update() {
        if (powerDistribution.isEmpty()) return;

        PowerDistribution pd = powerDistribution.get();

        buffer.voltage_V = pd.getVoltage();
        buffer.connected = buffer.voltage_V != 0;

        if (buffer.connected) {
            buffer.totalCurrent_A = pd.getTotalCurrent();
            buffer.currents_A = pd.getAllCurrents();

            PowerDistributionFaults faults = pd.getFaults();
            for (int i = 0; i < buffer.breakersTripped.length; i++) {
                buffer.breakersTripped[i] = faults.getBreakerFault(i);
            }
        } else {
            buffer.voltage_V = Double.NaN;
            buffer.totalCurrent_A = Double.NaN;
        }

        connWriter.set(buffer.connected);
        canWriter.set(buffer.connected);
        disconnAlert.set(!buffer.connected);
        voltageWriter.set(buffer.voltage_V);
        totalCurrentWriter.set(buffer.totalCurrent_A);
        currentsWriter.set(buffer.connected ? buffer.currents_A : null);
        breakersWriter.set(buffer.connected ? buffer.breakersTripped : null);
    }

    public boolean isBreakerTripped(int channel) {
        if (powerDistribution.isEmpty() || !buffer.connected || channel < 0 || channel > buffer.breakersTripped.length)
            return false;

        return buffer.breakersTripped[channel];
    }
}

package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawPublisher;
import edu.wpi.first.networktables.RawTopic;
import edu.wpi.first.util.datalog.RawLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import frc.robot.telemetry.Telemetry;
import java.util.Arrays;

public class RawWriter implements AutoCloseable {
    private RawPublisher ntPublisher_nl;
    private RawLogEntry logEntry_nl;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private byte[] currValue_nl;
    private final byte[] nullFallback = new byte[] {};

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public RawWriter(
            String typeString, String unit_nl, String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            RawTopic ntTopic = NetworkTableInstance.getDefault().getRawTopic(key);
            ntPublisher_nl = ntTopic.publish(typeString);
            logEntry_nl = null;

            if (unit_nl != null) {
                try {
                    ntTopic.setProperty("unit", '"' + unit_nl + '"');
                } catch (IllegalArgumentException e) {
                    Telemetry.reportWarning(e, true);
                }
            }
        } else {
            ntPublisher_nl = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry_nl = new RawLogEntry(DataLogManager.getLog(), NetworkTable.normalizeKey("NT:/" + key, false));
            } else {
                logEntry_nl = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public boolean set(byte[] val_nl) {
        if (ntPublisher_nl == null && logEntry_nl == null) return false;

        if (val_nl == null) val_nl = nullFallback;

        if (!includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val_nl);
        }

        if (!disableChecks && hasValue && Arrays.equals(val_nl, currValue_nl)) return false;

        if (includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val_nl);
        }
        if (logEntry_nl != null) {
            logEntry_nl.append(val_nl);
        }

        if (!disableChecks) {
            currValue_nl = val_nl.clone();
            hasValue = true;
        }

        return true;
    }

    @Override
    public void close() {
        // if (ntPublisher != null) {
        //     // NetworkTableInstance.getDefault().flushLocal(); // ensure values are published before closing
        // publisher
        //     ntPublisher.close();
        //     ntPublisher = null;
        // }
        // if(logEntry != null) {
        //     logEntry.finish();
        //     logEntry = null;
        // }
        logEntry_nl = null;
    }
}

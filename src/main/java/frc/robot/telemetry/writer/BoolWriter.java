package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import frc.robot.telemetry.Telemetry;

public class BoolWriter implements AutoCloseable {
    private BooleanPublisher ntPublisher;
    private BooleanLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private boolean currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public BoolWriter(String key, String unit, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            BooleanTopic ntTopic = NetworkTableInstance.getDefault().getBooleanTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;

            if (unit != null) {
                try {
                    ntTopic.setProperty("unit", '"' + unit + '"');
                } catch (IllegalArgumentException e) {
                    Telemetry.reportWarning(e, true);
                }
            }
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new BooleanLogEntry(DataLogManager.getLog(), NetworkTable.normalizeKey("NT:/" + key, false));
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public boolean set(boolean val) {
        if (ntPublisher == null && logEntry == null) return false;

        if (!includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }

        if (!disableChecks && hasValue && val == currValue) return false;

        if (includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }
        if (logEntry != null) {
            logEntry.append(val);
        }

        if (!disableChecks) {
            currValue = val;
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
        logEntry = null;
    }
}

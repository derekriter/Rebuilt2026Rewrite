package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;

public class BoolWriter implements AutoCloseable {
    private final BooleanPublisher ntPublisher;
    private final BooleanLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private boolean currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public BoolWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            BooleanTopic ntTopic = NetworkTableInstance.getDefault().getBooleanTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new BooleanLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public void set(boolean val) {
        if (!includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }

        if (!disableChecks && hasValue && val == currValue) return;

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
    }

    @Override
    public void close() {
        if (ntPublisher != null) {
            ntPublisher.close();
        }
    }
}

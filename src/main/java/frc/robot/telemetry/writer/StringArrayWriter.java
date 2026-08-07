package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.networktables.StringArrayTopic;
import edu.wpi.first.util.datalog.StringArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import java.util.Arrays;

public class StringArrayWriter implements AutoCloseable {
    private final StringArrayPublisher ntPublisher;
    private final StringArrayLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private String[] currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public StringArrayWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            StringArrayTopic ntTopic = NetworkTableInstance.getDefault().getStringArrayTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new StringArrayLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public void set(String[] val) {
        if (val == null) val = new String[] {};

        if (!includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }

        if (!disableChecks && hasValue && Arrays.equals(val, currValue)) return;

        if (includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }
        if (logEntry != null) {
            logEntry.append(val);
        }

        if (!disableChecks) {
            currValue = val.clone();
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

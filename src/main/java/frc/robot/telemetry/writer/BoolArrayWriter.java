package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.BooleanArrayPublisher;
import edu.wpi.first.networktables.BooleanArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import java.util.Arrays;

public class BoolArrayWriter implements AutoCloseable {
    private final BooleanArrayPublisher ntPublisher;
    private final BooleanArrayLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private boolean[] currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public BoolArrayWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            BooleanArrayTopic ntTopic = NetworkTableInstance.getDefault().getBooleanArrayTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new BooleanArrayLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public void set(boolean[] val) {
        if (val == null) val = new boolean[] {};

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

package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;

public class LongWriter implements AutoCloseable {
    private IntegerPublisher ntPublisher;
    private IntegerLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private long currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public LongWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            IntegerTopic ntTopic = NetworkTableInstance.getDefault().getIntegerTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new IntegerLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public boolean set(long val) {
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

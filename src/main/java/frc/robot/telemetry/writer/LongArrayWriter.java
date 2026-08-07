package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.IntegerArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.IntegerArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import java.util.Arrays;

public class LongArrayWriter implements AutoCloseable {
    private final IntegerArrayPublisher ntPublisher;
    private final IntegerArrayLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private long[] currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public LongArrayWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            IntegerArrayTopic ntTopic = NetworkTableInstance.getDefault().getIntegerArrayTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new IntegerArrayLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public void set(long[] val) {
        if (val == null) val = new long[] {};

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

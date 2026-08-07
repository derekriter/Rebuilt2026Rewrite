package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.FloatArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import java.util.Arrays;

public class FloatArrayWriter implements AutoCloseable {
    private final FloatArrayPublisher ntPublisher;
    private final FloatArrayLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private float[] currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public FloatArrayWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            FloatArrayTopic ntTopic = NetworkTableInstance.getDefault().getFloatArrayTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new FloatArrayLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public void set(float[] val) {
        if (val == null) val = new float[] {};

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

package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import java.util.Arrays;

public class DoubleArrayWriter implements AutoCloseable {
    private DoubleArrayPublisher ntPublisher;
    private DoubleArrayLogEntry logEntry;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private double[] currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public DoubleArrayWriter(String key, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            DoubleArrayTopic ntTopic = NetworkTableInstance.getDefault().getDoubleArrayTopic(key);
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), key);
            } else {
                logEntry = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public boolean set(double[] val) {
        if (ntPublisher == null && logEntry == null) return false;

        if (val == null) val = new double[] {};

        if (!includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }

        if (!disableChecks && hasValue && Arrays.equals(val, currValue)) return false;

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

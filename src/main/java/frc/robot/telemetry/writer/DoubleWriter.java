package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import frc.robot.telemetry.Telemetry;

public class DoubleWriter implements AutoCloseable {
    private DoublePublisher ntPublisher_nl;
    private DoubleLogEntry logEntry_nl;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;

    private boolean hasValue;
    private double currValue;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public DoubleWriter(String key, String unit_nl, boolean _includeNTInChecks, boolean _disableChecks) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            DoubleTopic ntTopic = NetworkTableInstance.getDefault().getDoubleTopic(key);
            ntPublisher_nl = ntTopic.publish();
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
                logEntry_nl =
                        new DoubleLogEntry(DataLogManager.getLog(), NetworkTable.normalizeKey("NT:/" + key, false));
            } else {
                logEntry_nl = null;
            }
        }

        hasValue = false;
        includeNTInChecks = _includeNTInChecks;
        disableChecks = _disableChecks;
    }

    public boolean set(double val) {
        if (ntPublisher_nl == null && logEntry_nl == null) return false;

        if (!includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val);
        }

        if (!disableChecks && hasValue && val == currValue) return false;

        if (includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val);
        }
        if (logEntry_nl != null) {
            logEntry_nl.append(val);
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
        logEntry_nl = null;
    }
}

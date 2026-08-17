package frc.robot.telemetry.writer;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.config.TelemetryConfig;
import frc.robot.util.CloneOperation;
import frc.robot.util.EqualityTest;

public class StructWriter<T extends StructSerializable> implements AutoCloseable {
    private StructPublisher<T> ntPublisher_nl;
    private StructLogEntry<T> logEntry_nl;
    private final EqualityTest<T> isEqual;
    private final CloneOperation<T> clone;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;
    private final T nullFallback;

    private boolean hasValue;
    private T currValue_nl;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public <S extends Struct<T>> StructWriter(
            S struct,
            String key,
            EqualityTest<T> _isEqual,
            CloneOperation<T> _clone,
            boolean _includeNTInChecks,
            boolean _disableChecks,
            T _nullFallback) {
        if (TelemetryConfig.telemetryLevel.logToNT) {
            StructTopic<T> ntTopic = NetworkTableInstance.getDefault().getStructTopic(key, struct);
            ntPublisher_nl = ntTopic.publish();
            logEntry_nl = null;
        } else {
            ntPublisher_nl = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry_nl = StructLogEntry.create(DataLogManager.getLog(), key, struct);
            } else {
                logEntry_nl = null;
            }
        }

        if (_isEqual == null || _clone == null) {
            isEqual = null;
            clone = null;
            hasValue = false;
            includeNTInChecks = false;
            disableChecks = true;
        } else {
            isEqual = _isEqual;
            clone = _clone;
            hasValue = false;
            includeNTInChecks = _includeNTInChecks;
            disableChecks = _disableChecks;
        }

        nullFallback = _nullFallback;
    }

    public boolean set(T val_nl) {
        if (ntPublisher_nl == null && logEntry_nl == null) return false;

        if (val_nl == null) val_nl = nullFallback;

        if (!includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val_nl);
        }

        if (!disableChecks && hasValue && isEqual != null && isEqual.test(val_nl, currValue_nl)) return false;

        if (includeNTInChecks && ntPublisher_nl != null) {
            ntPublisher_nl.set(val_nl);
        }
        if (logEntry_nl != null) {
            logEntry_nl.append(val_nl);
        }

        if (!disableChecks && clone != null) {
            currValue_nl = clone.clone(val_nl);
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

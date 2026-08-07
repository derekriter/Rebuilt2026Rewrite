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
    private final StructPublisher<T> ntPublisher;
    private final StructLogEntry<T> logEntry;
    private final EqualityTest<T> isEqual;
    private final CloneOperation<T> clone;
    private final boolean includeNTInChecks;
    private final boolean disableChecks;
    private final T nullFallback;

    private boolean hasValue;
    private T currValue;

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
            ntPublisher = ntTopic.publish();
            logEntry = null;
        } else {
            ntPublisher = null;

            if (TelemetryConfig.telemetryLevel.logToFile) {
                logEntry = StructLogEntry.create(DataLogManager.getLog(), key, struct);
            } else {
                logEntry = null;
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

    public void set(T val) {
        if (val == null) val = nullFallback;

        if (!includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }

        if (!disableChecks && hasValue && isEqual != null && isEqual.test(val, currValue)) return;

        if (includeNTInChecks && ntPublisher != null) {
            ntPublisher.set(val);
        }
        if (logEntry != null) {
            logEntry.append(val);
        }

        if (!disableChecks && clone != null) {
            currValue = clone.clone(val);
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

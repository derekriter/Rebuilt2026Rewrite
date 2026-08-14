package frc.robot.telemetry.writer.compound;

import frc.robot.subsystems.launcher.ShooterTarget;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;

public class ShooterTargetWriter implements AutoCloseable {

    private final DoubleWriter velocityWriter;
    private final DoubleWriter distanceWriter;
    private final BoolWriter isLegalWriter;

    private final ShooterTarget nullFallback;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public ShooterTargetWriter(
            String table, boolean includeNTInChecks, boolean disableChecks, ShooterTarget _nullFallback) {
        velocityWriter =
                Telemetry.makeDoubleWriterEx(table, "velocity", TelemetryUnits.rpm, includeNTInChecks, disableChecks);
        distanceWriter = Telemetry.makeDoubleWriterEx(
                table, "distance", TelemetryUnits.meters, includeNTInChecks, disableChecks);
        isLegalWriter = Telemetry.makeBoolWriterEx(table, "isLegal", null, includeNTInChecks, disableChecks);

        nullFallback = _nullFallback;
    }

    public void set(ShooterTarget target) {
        if (target == null) target = nullFallback;

        velocityWriter.set(target.asShooterRPM());
        distanceWriter.set(target.asMetersToTarget());
        isLegalWriter.set(target.isLegal());
    }

    @Override
    public void close() {
        velocityWriter.close();
        distanceWriter.close();
        isLegalWriter.close();
    }
}

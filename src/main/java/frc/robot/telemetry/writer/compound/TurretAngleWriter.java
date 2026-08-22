package frc.robot.telemetry.writer.compound;

import frc.robot.subsystems.launcher.TurretAngle;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.BoolWriter;
import frc.robot.telemetry.writer.DoubleWriter;

public class TurretAngleWriter implements AutoCloseable {

    private final DoubleWriter motorRotsWriter;
    private final DoubleWriter mechDegWriter;
    private final BoolWriter isWrappedWriter;
    private final BoolWriter isLegalWriter;

    private final TurretAngle nullFallback;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public TurretAngleWriter(
            String table, boolean includeNTInChecks, boolean disableChecks, TurretAngle _nullFallback) {
        motorRotsWriter = Telemetry.makeDoubleWriterEx(
                table, "motorRots", TelemetryUnits.rotations, includeNTInChecks, disableChecks);
        mechDegWriter = Telemetry.makeDoubleWriterEx(
                table, "mechDeg", TelemetryUnits.degrees, includeNTInChecks, disableChecks);
        isWrappedWriter = Telemetry.makeBoolWriterEx(table, "isWrapped", null, includeNTInChecks, disableChecks);
        isLegalWriter = Telemetry.makeBoolWriterEx(table, "isLegal", null, includeNTInChecks, disableChecks);

        nullFallback = _nullFallback;
    }

    public void set(TurretAngle angle_nl) {
        if (angle_nl == null) angle_nl = nullFallback;

        motorRotsWriter.set(angle_nl.asMotorRotations());
        mechDegWriter.set(angle_nl.asMechanismDegrees());
        isWrappedWriter.set(angle_nl.isWrapped());
        isLegalWriter.set(angle_nl.isLegal());
    }

    @Override
    public void close() {
        motorRotsWriter.close();
        mechDegWriter.close();
        isWrappedWriter.close();
        isLegalWriter.close();
    }
}

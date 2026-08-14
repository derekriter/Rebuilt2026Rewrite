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
        isLegalWriter = Telemetry.makeBoolWriterEx(table, "isLegel", null, includeNTInChecks, disableChecks);

        nullFallback = _nullFallback;
    }

    public void set(TurretAngle angle) {
        if (angle == null) angle = nullFallback;

        motorRotsWriter.set(angle.asMotorRotations());
        mechDegWriter.set(angle.asMechanismDegrees());
        isWrappedWriter.set(angle.isWrapped());
        isLegalWriter.set(angle.isLegal());
    }

    @Override
    public void close() {
        motorRotsWriter.close();
        mechDegWriter.close();
        isWrappedWriter.close();
        isLegalWriter.close();
    }
}

package frc.robot.telemetry.writer.compound;

import frc.robot.subsystems.swerve.SwerveReport;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.BoolWriter;

public class SwerveReportWriter implements AutoCloseable {

    private final BoolWriter isOperationalWriter;

    private final SwerveReport nullFallback;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public SwerveReportWriter(String table, SwerveReport _nullFallback) {
        isOperationalWriter = Telemetry.makeBoolWriterEx(table, "isOperational", null, true, false);

        nullFallback = _nullFallback;
    }

    public void set(SwerveReport report) {
        if (report == null) report = nullFallback;

        isOperationalWriter.set(report.isOperational);
    }

    @Override
    public void close() {
        isOperationalWriter.close();
    }
}

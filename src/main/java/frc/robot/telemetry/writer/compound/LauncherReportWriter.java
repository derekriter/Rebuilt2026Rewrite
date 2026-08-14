package frc.robot.telemetry.writer.compound;

import frc.robot.subsystems.launcher.LauncherReport;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.BoolWriter;

public class LauncherReportWriter implements AutoCloseable {

    private final BoolWriter turretOperationalWriter;

    private final BoolWriter shooterOperationWriter;
    private final BoolWriter shooterIsAtTargetWriter;

    private final LauncherReport nullFallback;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public LauncherReportWriter(String table, LauncherReport _nullFallback) {
        turretOperationalWriter = Telemetry.makeBoolWriterEx(table, "turretOperational", null, true, false);

        shooterOperationWriter = Telemetry.makeBoolWriterEx(table, "shooterOperational", null, true, false);
        shooterIsAtTargetWriter = Telemetry.makeBoolWriterEx(table, "shooterIsAtTarget", null, true, false);

        nullFallback = _nullFallback;
    }

    public void set(LauncherReport report) {
        if (report == null) report = nullFallback;

        turretOperationalWriter.set(report.turretOperational);

        shooterOperationWriter.set(report.shooterOperational);
        shooterIsAtTargetWriter.set(report.shooterIsAtTarget);
    }

    @Override
    public void close() {
        turretOperationalWriter.close();
        shooterOperationWriter.close();
        shooterIsAtTargetWriter.close();
    }
}

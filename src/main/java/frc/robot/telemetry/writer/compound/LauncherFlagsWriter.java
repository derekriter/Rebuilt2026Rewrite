package frc.robot.telemetry.writer.compound;

import frc.robot.subsystems.launcher.LauncherFlags;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.BoolWriter;

public class LauncherFlagsWriter implements AutoCloseable {

    private final BoolWriter turretOperationalWriter;

    private final BoolWriter shooterOperationWriter;
    private final BoolWriter shooterIsAtTargetWriter;

    /**
     * DO NOT USE OUTSIDE Telemetry.java!!!
     */
    public LauncherFlagsWriter(String table) {
        turretOperationalWriter = Telemetry.makeBoolWriterEx(table, "turretOperation", null, true, false);

        shooterOperationWriter = Telemetry.makeBoolWriterEx(table, "shooterOperation", null, true, false);
        shooterIsAtTargetWriter = Telemetry.makeBoolWriterEx(table, "shooterIsAtTarget", null, true, false);
    }

    public void update(LauncherFlags flags) {
        turretOperationalWriter.set(flags.turretOperational);

        shooterOperationWriter.set(flags.shooterOperational);
        shooterIsAtTargetWriter.set(flags.shooterIsAtTarget);
    }

    @Override
    public void close() {
        turretOperationalWriter.close();
        shooterOperationWriter.close();
        shooterIsAtTargetWriter.close();
    }
}

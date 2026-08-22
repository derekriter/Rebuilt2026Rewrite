package frc.robot.config;

import frc.robot.telemetry.Telemetry;

public final class Overrides {
    public static final boolean disableShooterSafety = false;
    public static final boolean disableShooter = false;

    public static final boolean disableTurretSafety = false;
    public static final boolean disableTurret = false;

    public static final boolean disablePDHMonitoring = false;

    public static final boolean disableLEDs = false;

    public static final boolean disableSwerveSafety = false;

    public static void telemeterizeOverrides() {
        Telemetry.makeBoolWriterInitialEx("Overrides", "disableShooterSafety", null, disableShooterSafety, false, true)
                .close();
        Telemetry.makeBoolWriterInitialEx("Overrides", "disableShooter", null, disableShooter, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx("Overrides", "disableTurretSafety", null, disableTurretSafety, false, true)
                .close();
        Telemetry.makeBoolWriterInitialEx("Overrides", "disableTurret", null, disableTurret, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx("Overrides", "disablePDHMonitoring", null, disablePDHMonitoring, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx("Overrides", "disabledLEDs", null, disableLEDs, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx("Overrides", "disableSwerveSafety", null, disableSwerveSafety, false, true)
                .close();
    }

    private Overrides() {}
}

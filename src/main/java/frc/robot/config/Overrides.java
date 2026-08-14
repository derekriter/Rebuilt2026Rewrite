package frc.robot.config;

import frc.robot.telemetry.Telemetry;

public final class Overrides {
    public static final boolean disableShooterSafety = false;
    public static final boolean disableShooter = false;

    public static final boolean disableTurretSafety = false;
    public static final boolean disableTurret = false;

    public static final boolean disablePDHMonitoring = false;

    public static final boolean disableLEDs = false;

    public static void telemeterizeOverrides() {
        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(),
                        "disableShooterSafety",
                        null,
                        disableShooterSafety,
                        false,
                        true)
                .close();
        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(), "disableShooter", null, disableShooter, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(), "disableTurretSafety", null, disableTurretSafety, false, true)
                .close();
        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(), "disableTurret", null, disableTurret, false, true)
                .close();

        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(),
                        "disablePDHMonitoring",
                        null,
                        disablePDHMonitoring,
                        false,
                        true)
                .close();

        Telemetry.makeBoolWriterInitialEx(
                        Overrides.class.getSimpleName(), "disabledLEDs", null, disableLEDs, false, true)
                .close();
    }

    private Overrides() {}
}

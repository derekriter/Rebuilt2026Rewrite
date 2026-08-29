package frc.robot.constants;

import org.littletonrobotics.junction.Logger;

public final class Overrides {
    public static final boolean disableShooterSafety = false;
    public static final boolean disableShooter = false;

    public static final boolean disableTurretSafety = false;
    public static final boolean disableTurret = false;

    public static final boolean disablePDHMonitoring = false;

    public static final boolean disableLEDs = false;

    public static final boolean disableSwerveSafety = false;

    public static void telemeterizeOverrides() {
        Logger.recordOutput("Overrides/disableShooterSafety", disableShooterSafety);
        Logger.recordOutput("Overrides/disableShooter", disableShooter);

        Logger.recordOutput("Overrides/disableTurretSafety", disableTurretSafety);
        Logger.recordOutput("Overrides/disableTurret", disableTurret);

        Logger.recordOutput("Overrides/disablePDHMonitoring", disablePDHMonitoring);

        Logger.recordOutput("Overrides/disableLEDs", disableLEDs);

        Logger.recordOutput("Overrides/disableSwerveSafety", disableSwerveSafety);
    }

    private Overrides() {}
}

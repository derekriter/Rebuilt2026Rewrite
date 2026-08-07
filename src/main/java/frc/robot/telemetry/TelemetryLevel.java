package frc.robot.telemetry;

public enum TelemetryLevel {
    ENABLED(true, true),
    NT_ONLY(true, false),
    FILE_ONLY(false, true),
    DISABLED(false, false);

    public final boolean logToNT;
    public final boolean logToFile;

    TelemetryLevel(boolean _logToNT, boolean _logToFile) {
        logToNT = _logToNT;
        logToFile = _logToFile;
    }
}

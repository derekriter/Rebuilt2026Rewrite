package frc.robot.telemetry;

public enum TelemetryLevel {
    ENABLED(true, true),
    NT_AND_CONSOLE(true, false),
    FILE_AND_CONSOLE(false, true),
    CONSOLE_ONLY(false, false);

    public final boolean logToNT;
    public final boolean logToFile;

    TelemetryLevel(boolean _logToNT, boolean _logToFile) {
        logToNT = _logToNT;
        logToFile = _logToFile;
    }
}

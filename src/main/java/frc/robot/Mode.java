package frc.robot;

public enum Mode {
    REAL,
    SIM,
    REPLAY;

    private static final Mode simMode = SIM;

    public static Mode getMode() {
        return Robot.isReal() ? REAL : simMode;
    }
}

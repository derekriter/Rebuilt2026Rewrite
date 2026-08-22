package frc.robot.brain;

public enum OpMode {
    DISABLED(false, false),
    TELEOP(true, true),
    AUTON(true, false),
    TEST(true, true);

    public final boolean enabled;
    public final boolean driverControlled;

    private OpMode(boolean _enabled, boolean _driverControlled) {
        this.enabled = _enabled;
        this.driverControlled = _driverControlled;
    }
}

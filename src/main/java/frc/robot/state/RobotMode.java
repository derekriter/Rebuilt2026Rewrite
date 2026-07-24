package frc.robot.state;

public enum RobotMode {
    DISABLED(false, false),
    TELEOP(true, true),
    AUTON(true, false),
    TEST(true, true);

    public final boolean enabled;
    public final boolean driverControlled;

    private RobotMode(boolean _enabled, boolean _driverControlled) {
        this.enabled = _enabled;
        this.driverControlled = _driverControlled;
    }
}

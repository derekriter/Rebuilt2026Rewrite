package frc.robot.config;

import edu.wpi.first.wpilibj.RobotBase;

public enum RobotMode {
    REAL,
    SIM,
    REPLAY;

    public static final RobotMode simMode = SIM;
    public static final RobotMode currentMode = RobotBase.isReal() ? REAL : simMode;
}

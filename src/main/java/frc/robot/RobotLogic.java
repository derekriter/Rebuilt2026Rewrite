package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.state.RobotMode;
import frc.robot.state.RobotState;
import java.util.Optional;

public class RobotLogic {

    private RobotState state;
    private Optional<RobotState> lastState;

    public RobotLogic(RobotState _state, Optional<RobotState> _lastState) {
        state = _state;
        lastState = _lastState;
    }

    public void updateState() {
        if (DriverStation.isTeleopEnabled()) {
            state.robotMode = RobotMode.TELEOP;
        } else if (DriverStation.isAutonomousEnabled()) {
            state.robotMode = RobotMode.AUTON;
        } else if (DriverStation.isTestEnabled()) {
            state.robotMode = RobotMode.TEST;
        } else {
            state.robotMode = RobotMode.DISABLED;
        }
    }

    public void firstPeriodic() {}

    public void disabledInit() {}

    public void disabledPeriodic() {}

    public void disabledExit() {}

    public void enabledInit() {}

    public void enabledPeriodic() {}

    public void enabledExit() {}

    public void autonomousInit() {}

    public void autonomousPeriodic() {}

    public void autonomousExit() {}

    public void teleopInit() {}

    public void teleopPeriodic() {}

    public void teleopExit() {}

    public void testInit() {}

    public void testPeriodic() {}

    public void testExit() {}

    public void finalPeriodic() {}
}

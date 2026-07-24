package frc.robot.state;

public class RobotState {
    public RobotMode robotMode;
    public boolean isReal;

    public void copyFrom(RobotState ref) {
        robotMode = ref.robotMode;
        isReal = ref.isReal;
    }
}

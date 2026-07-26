package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.config.Overrides;

public class RobotBrain {

    private Robot robot;
    private Timer modeTimer;

    public RobotBrain(Robot _robot) {
        robot = _robot;
        modeTimer = new Timer();
    }

    public void pollState() {
        if (DriverStation.isTeleopEnabled()) {
            robot.state.opMode = OpMode.TELEOP;
        } else if (DriverStation.isAutonomousEnabled()) {
            robot.state.opMode = OpMode.AUTON;
        } else if (DriverStation.isTestEnabled()) {
            robot.state.opMode = OpMode.TEST;
        } else {
            robot.state.opMode = OpMode.DISABLED;
        }

        if (robot.lastState.isEmpty() || robot.state.opMode != robot.lastState.get().opMode) {
            robot.state.isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            modeTimer.restart();
            robot.state.autoWinnerIsKnown = false;
        }

        robot.state.modeTime.mut_replace(modeTimer.get(), Seconds);
        robot.state.fieldZone =
                FieldZone.fromRobotX(robot.container.swerve.getPose().getMeasureX());

        if (robot.state.opMode == OpMode.TELEOP) {
            if (!robot.state.autoWinnerIsKnown) {
                String gameMessage = DriverStation.getGameSpecificMessage();

                if (gameMessage != null && !gameMessage.isEmpty()) {
                    if (gameMessage.charAt(0) == 'B') {
                        robot.state.autoWinnerIsKnown = true;
                        robot.state.didWinAuto = !robot.state.isRed;
                    } else if (gameMessage.charAt(0) == 'R') {
                        robot.state.autoWinnerIsKnown = true;
                        robot.state.didWinAuto = robot.state.isRed;
                    }
                }
            }

            robot.state.phase = TeleopPhase.fromTeleopTimer(robot.state.modeTime);
            robot.state.timeLeftInPhase.mut_replace(robot.state.phase.getTimeRemaining(robot.state.modeTime));
        }

        robot.container.launcher.pollFlags(robot.state.launcherFlags);
    }

    public void determineModes() {
        switch (robot.state.opMode) {
            case DISABLED, TEST -> {
                robot.state.targetingMode = TargetingMode.DISABLED;
            }
            case TELEOP -> {
                if (Overrides.disableShooter) {
                    robot.state.targetingMode = TargetingMode.DISABLED;
                } else if (!robot.state.isTurretHomed && !Overrides.disableTurret) {
                    robot.state.targetingMode = TargetingMode.HOMING;
                } else if (robot.state.fieldZone == FieldZone.RED && robot.state.isRed
                        || robot.state.fieldZone == FieldZone.BLUE && !robot.state.isRed) {
                    robot.state.targetingMode = TargetingMode.TARGETING_HUB;
                } else {
                    robot.state.targetingMode = TargetingMode.TARGETING_FZONE;
                }
            }
            case AUTON -> {
                if (Overrides.disableShooter || Overrides.disableTurret) {
                    robot.state.targetingMode = TargetingMode.DISABLED;
                } else if (!robot.state.isTurretHomed) {
                    robot.state.targetingMode = TargetingMode.HOMING;
                } else {
                    robot.state.targetingMode = TargetingMode.TARGETING_HUB;
                }
            }
        }
    }

    public void runCommands() {
        if (robot.lastState.isEmpty() || robot.state.targetingMode != robot.lastState.get().targetingMode) {
            switch (robot.state.targetingMode) {
                case TARGETING_HUB -> {}
                case TARGETING_FZONE -> {}
                case FIXED_TARGET -> {}
                case HOMING -> {
                    removeSubsystemDefaultCommand(robot.container.launcher);
                    CommandScheduler.getInstance().schedule(robot.container.homeLauncherCmd.finallyDo(interrupted -> {
                        if (!interrupted) robot.state.isTurretHomed = true;
                    }));
                }
                case DISABLED -> removeSubsystemDefaultCommand(robot.container.launcher);
            }
        }
    }

    private void changeSubsystemDefaultCommand(Subsystem sub, Command cmd) {
        Command prevDefault = sub.getDefaultCommand();
        Command curr = sub.getCurrentCommand();

        if (curr != null && curr.equals(prevDefault)) {
            curr.cancel();
        }
        sub.setDefaultCommand(cmd);
    }

    private void removeSubsystemDefaultCommand(Subsystem sub) {
        Command prevDefault = sub.getDefaultCommand();
        Command curr = sub.getCurrentCommand();

        if (curr != null && curr.equals(prevDefault)) {
            curr.cancel();
        }
        sub.removeDefaultCommand();
    }
}

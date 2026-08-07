package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.config.ControllerConfig;
import frc.robot.config.Overrides;
import java.util.Optional;

public class RobotBrain {

    public RobotState state;
    public Optional<RobotState> lastState;
    private Timer modeTimer;

    public RobotBrain() {
        lastState = Optional.empty();
        state = new RobotState();
        state.isReal = Robot.isReal();

        modeTimer = new Timer();
    }

    public void pollState() {
        if (DriverStation.isTeleopEnabled()) {
            state.opMode = OpMode.TELEOP;
        } else if (DriverStation.isAutonomousEnabled()) {
            state.opMode = OpMode.AUTON;
        } else if (DriverStation.isTestEnabled()) {
            state.opMode = OpMode.TEST;
        } else {
            state.opMode = OpMode.DISABLED;
        }

        if (lastState.isEmpty() || state.opMode != lastState.get().opMode) {
            state.isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
            modeTimer.restart();
            state.autoWinnerIsKnown = false;
        }

        state.modeTime.mut_replace(modeTimer.get(), Seconds);
        state.fieldZone =
                FieldZone.fromRobotX(RobotContainer.instance().swerve.getPose().getMeasureX());

        if (state.opMode == OpMode.TELEOP) {
            if (!state.autoWinnerIsKnown) {
                String gameMessage = DriverStation.getGameSpecificMessage();

                if (gameMessage != null && !gameMessage.isEmpty()) {
                    if (gameMessage.charAt(0) == 'B') {
                        state.autoWinnerIsKnown = true;
                        state.didWinAuto = !state.isRed;
                    } else if (gameMessage.charAt(0) == 'R') {
                        state.autoWinnerIsKnown = true;
                        state.didWinAuto = state.isRed;
                    }
                }
            }

            state.phase = TeleopPhase.fromTeleopTimer(state.modeTime);
            state.timeLeftInPhase.mut_replace(state.phase.getTimeRemaining(state.modeTime));
        }

        RobotContainer.instance().launcher.pollFlags(state.launcherFlags);

        boolean driver2IsMovingJoysticks =
                Math.abs(RobotContainer.instance().driver2.getLeftX()) > ControllerConfig.overrideTurretThreshold
                        || Math.abs(RobotContainer.instance().driver2.getRightX())
                                > ControllerConfig.overrideTurretThreshold;
        boolean driver2IsPressingJoysticks = RobotContainer.instance().driver1.getLeftStickButton()
                || RobotContainer.instance().driver2.getRightStickButton();
        state.overrideTurret = (lastState.map(ls -> ls.overrideTurret).orElse(false) || driver2IsMovingJoysticks)
                && !driver2IsPressingJoysticks;

        if (RobotContainer.instance().driver2.getXButtonPressed()) {
            state.isTurretHomed = false;
        }
    }

    public void determineModes() {
        boolean shooterCanRun = !Overrides.disableShooter && state.launcherFlags.shooterOperational;
        boolean turretCanRun = !Overrides.disableTurret && state.launcherFlags.turretOperational;

        switch (state.opMode) {
            case DISABLED, TEST -> {
                state.targetingMode = TargetingMode.DISABLED;
            }
            case TELEOP -> {
                if (!shooterCanRun) {
                    state.targetingMode = TargetingMode.DISABLED;
                } else if (!state.isTurretHomed && turretCanRun) {
                    state.targetingMode = TargetingMode.HOMING;
                } else if (state.fieldZone == FieldZone.RED && state.isRed
                        || state.fieldZone == FieldZone.BLUE && !state.isRed) {
                    state.targetingMode = TargetingMode.TARGETING_HUB;
                } else {
                    state.targetingMode = TargetingMode.TARGETING_FZONE;
                }
            }
            case AUTON -> {
                if (Overrides.disableShooter || Overrides.disableTurret) {
                    state.targetingMode = TargetingMode.DISABLED;
                } else if (!state.isTurretHomed) {
                    state.targetingMode = TargetingMode.HOMING;
                } else {
                    state.targetingMode = TargetingMode.TARGETING_HUB;
                }
            }
        }
    }

    public void runCommands() {
        if (lastState.isEmpty() || state.targetingMode != lastState.get().targetingMode) {
            switch (state.targetingMode) {
                case TARGETING_HUB -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().launcher,
                        RobotContainer.instance().aimAtHubCmd());
                case TARGETING_FZONE -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().launcher,
                        RobotContainer.instance().aimAtFZoneCmd());
                case HOMING -> {
                    removeSubsystemDefaultCommand(RobotContainer.instance().launcher);
                    CommandScheduler.getInstance()
                            .schedule(
                                    RobotContainer.instance().homeLauncherCmd().finallyDo(interrupted -> {
                                        if (!interrupted) state.isTurretHomed = true;
                                    }));
                }
                case DISABLED -> removeSubsystemDefaultCommand(RobotContainer.instance().launcher);
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

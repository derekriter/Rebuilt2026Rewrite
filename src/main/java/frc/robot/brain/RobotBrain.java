package frc.robot.brain;

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
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.writer.compound.RobotStateWriter;
import java.util.Optional;

public class RobotBrain {

    public static final RobotStateWriter robotStateWriter = Telemetry.makeRobotStateWriter("RobotBrain", "robotState");

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
        pollOpModeAndAlliance();

        RobotContainer.instance().pdh.update();
        RobotContainer.instance().launcher.report(state.launcherReport);
        RobotContainer.instance().swerve.report(state.swerveReport);
        RobotContainer.instance().leds.update();

        state.fieldZone = FieldZone.fromRobotX(
                RobotContainer.instance().swerve.getState().Pose.getX());

        if (state.opMode == OpMode.TELEOP) {
            pollTeleopData();
        }

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

    private void pollOpModeAndAlliance() {
        state.isDSAttached = DriverStation.isDSAttached();

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
            state.autoWinnerIsKnown = false;

            modeTimer.restart();
            state.phase = TeleopPhase.TRANSITION_SHIFT;
            state.timeLeftInPhase_s = Double.NaN;
            state.isHubActive = false;

            state.overrideTurret = false;
        }

        state.modeTime_s = modeTimer.get();
    }

    private void pollTeleopData() {
        if (!state.autoWinnerIsKnown) {
            String gameMessage = DriverStation.getGameSpecificMessage();

            if (gameMessage != null && !gameMessage.isEmpty()) {
                if (Character.toUpperCase(gameMessage.charAt(0)) == 'B') {
                    state.autoWinnerIsKnown = true;
                    state.didWinAuto = !state.isRed;
                } else if (Character.toUpperCase(gameMessage.charAt(0)) == 'R') {
                    state.autoWinnerIsKnown = true;
                    state.didWinAuto = state.isRed;
                }
            }
        }

        state.phase = TeleopPhase.fromTeleopTime(state.modeTime_s);
        state.timeLeftInPhase_s = state.phase.getTimeRemaining_s(state.modeTime_s);
        if (state.autoWinnerIsKnown) {
            state.isHubActive = state.phase.isHubEnabled(state.didWinAuto);
        } else {
            state.isHubActive = true;
        }
    }

    @SuppressWarnings("unused")
    public void determineModes() {
        boolean shooterCanRun = !Overrides.disableShooter && state.launcherReport.shooterOperational;
        boolean shooterError = !state.launcherReport.shooterOperational && !Overrides.disableShooter;

        boolean turretCanRun = !Overrides.disableTurret && state.launcherReport.turretOperational;
        boolean turretError = !state.launcherReport.turretOperational && !Overrides.disableTurret;

        boolean swerveError = !state.swerveReport.isOperational;

        boolean hardwareError = shooterError || turretError || swerveError;

        switch (state.opMode) {
            case DISABLED -> {
                state.targetingMode = TargetingMode.DISABLED;

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                } else if (state.isDSAttached) {
                    state.ledsMode = LEDsMode.IDLE;
                } else {
                    state.ledsMode = LEDsMode.DISCONNECTED;
                }
            }
            case TEST -> {
                state.targetingMode = TargetingMode.DISABLED;
                state.ledsMode = hardwareError ? LEDsMode.ERROR : LEDsMode.OK;
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

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                } else if (state.phase == TeleopPhase.ENDGAME) {
                    state.ledsMode = LEDsMode.ENDGAME;
                } else if (state.timeLeftInPhase_s <= 3) {
                    state.ledsMode = LEDsMode.SHIFT_WARNING;
                } else if (state.isHubActive) {
                    state.ledsMode = LEDsMode.HUB_ACTIVE;
                } else {
                    state.ledsMode = LEDsMode.HUB_INACTIVE;
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

                state.ledsMode = hardwareError ? LEDsMode.ERROR : LEDsMode.AUTON;
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

        if (lastState.isEmpty() || state.ledsMode != lastState.get().ledsMode) {
            switch (state.ledsMode) {
                case DISCONNECTED -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().disconnLEDsCmd());
                case IDLE -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().idleLEDsCmd());
                case AUTON -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().autonLEDsCmd());
                case OK -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().okLEDsCmd());
                case ERROR -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().errorLEDsCmd());
                case ENDGAME -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().endgameLEDsCmd());
                case SHIFT_WARNING -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().shiftWarningLEDsCmd());
                case HUB_ACTIVE -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().hubActiveLEDsCmd());
                case HUB_INACTIVE -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().leds,
                        RobotContainer.instance().hubInactiveLEDsCmd());
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

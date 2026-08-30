package frc.robot.brain;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.ControllerConstants;
import frc.robot.constants.Overrides;
import frc.robot.subsystems.Controller;
import frc.robot.util.Console;
import java.util.Optional;
import org.littletonrobotics.conduit.ConduitApi;
import org.littletonrobotics.junction.Logger;

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
        pollBasicInfo();

        RobotContainer.instance().launcher.report(state.launcherReport);
        RobotContainer.instance().swerve.report(state.swerveReport);
        RobotContainer.instance().intake.report(state.intakeReport);

        if (state.opMode == OpMode.TELEOP) {
            pollTeleopData();
        }

        pollDriverInput();
    }

    private void pollBasicInfo() {
        state.isDSAttached = DriverStation.isDSAttached();
        state.isFMSAttached = DriverStation.isFMSAttached();
        if (lastState.isEmpty() || state.isDSAttached != lastState.get().isDSAttached) {
            state.isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        }

        if (DriverStation.isTeleopEnabled()) {
            state.opMode = OpMode.TELEOP;
        } else if (DriverStation.isAutonomousEnabled()) {
            state.opMode = OpMode.AUTON;
        } else if (DriverStation.isTestEnabled()) {
            state.opMode = OpMode.TEST;
        } else {
            state.opMode = OpMode.DISABLED;
        }

        boolean changedMode = lastState.isEmpty() || state.opMode != lastState.get().opMode;

        if (changedMode || state.opMode == OpMode.DISABLED) {
            state.isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        }
        if (changedMode) {
            state.autoWinnerIsKnown = false;

            modeTimer.restart();
            state.phase = TeleopPhase.TRANSITION_SHIFT;
            state.timeLeftInPhase_s = Double.NaN;
            state.isHubActive = false;

            state.overrideTurret = false;
        }

        state.modeTime_s = modeTimer.get();
        state.isBrownedOut = ConduitApi.getInstance().getBrownedOut();
        state.fieldZone =
                FieldZone.fromRobotX(RobotContainer.instance().swerve.getPose().getX());
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

    private void pollDriverInput() {
        XboxController d2 = RobotContainer.instance().driver2.getHID();

        boolean driver2IsMovingJoysticks =
                Controller.isPastDeadband(d2.getLeftX(), ControllerConstants.overrideTurretThreshold)
                        || Controller.isPastDeadband(d2.getRightX(), ControllerConstants.overrideTurretThreshold);
        boolean driver2IsPressingJoysticks = d2.getLeftStickButton() || d2.getRightStickButton();

        if (state.opMode == OpMode.DISABLED) {
            state.overrideTurret = false;
        } else {
            state.overrideTurret = (lastState.map(ls -> ls.overrideTurret).orElse(false) || driver2IsMovingJoysticks)
                    && !driver2IsPressingJoysticks;
        }
    }

    @SuppressWarnings("unused")
    public void determineModes() {
        boolean shooterCanRun = !Overrides.disableShooter && state.launcherReport.shooterOperational;
        boolean shooterError = !state.launcherReport.shooterOperational && !Overrides.disableShooter;

        boolean turretCanRun = !Overrides.disableTurret && state.launcherReport.turretOperational;
        boolean turretError = !state.launcherReport.turretOperational && !Overrides.disableTurret;

        boolean swerveError = !state.swerveReport.isOperational;

        boolean deployerCanRun = !Overrides.disableIntake;
        boolean intakeError = !Overrides.disableIntake
                && !(state.intakeReport.rollerOperational && state.intakeReport.deployerOperational);

        boolean hardwareError = shooterError || turretError || swerveError || intakeError;

        switch (state.opMode) {
            case DISABLED -> {
                state.targetingMode = TargetingMode.DISABLED;
                state.driveMode = DriveMode.DISABLED;

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.FAST_PULSING;
                } else if (state.isDSAttached) {
                    state.ledsMode = LEDsMode.IDLE;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                } else {
                    state.ledsMode = LEDsMode.DISCONNECTED;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                }

                state.shouldDeployIntake = false;
            }
            case TEST -> {
                state.targetingMode = TargetingMode.DISABLED;
                state.driveMode = DriveMode.TELEOP;

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.FAST_PULSING;
                } else {
                    state.ledsMode = LEDsMode.OK;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                }

                state.shouldDeployIntake = false;
            }
            case TELEOP -> {
                if (!shooterCanRun) {
                    state.targetingMode = TargetingMode.DISABLED;
                } else if (!state.isTurretHomed && turretCanRun) {
                    state.targetingMode = TargetingMode.HOMING;
                } else if (isInFZone()) {
                    state.targetingMode = TargetingMode.TARGETING_HUB;
                } else {
                    state.targetingMode = TargetingMode.TARGETING_FZONE;
                }

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.FAST_PULSING;
                } else if (state.phase == TeleopPhase.ENDGAME) {
                    state.ledsMode = LEDsMode.ENDGAME;

                    if (state.modeTime_s - TeleopPhase.SHIFT4.endTime.in(Seconds) <= 5) {
                        state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.CONTINUOUS;
                    } else {
                        state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                    }
                } else if (state.timeLeftInPhase_s <= 3) {
                    state.ledsMode = LEDsMode.SHIFT_WARNING;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.KNOCK;
                } else if (state.isHubActive) {
                    state.ledsMode = LEDsMode.HUB_ACTIVE;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                } else {
                    state.ledsMode = LEDsMode.HUB_INACTIVE;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                }

                state.driveMode = DriveMode.TELEOP;
                state.shouldDeployIntake = !state.intakeReport.hasDeployed && deployerCanRun;
            }
            case AUTON -> {
                if (Overrides.disableShooter || Overrides.disableTurret) {
                    state.targetingMode = TargetingMode.DISABLED;
                } else if (!state.isTurretHomed) {
                    state.targetingMode = TargetingMode.HOMING;
                } else {
                    state.targetingMode = TargetingMode.TARGETING_HUB;
                }

                if (hardwareError) {
                    state.ledsMode = LEDsMode.ERROR;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.FAST_PULSING;
                } else {
                    state.ledsMode = LEDsMode.AUTON;
                    state.driver1RumbleMode = state.driver2RumbleMode = RumbleMode.IDLE;
                }

                state.driveMode = DriveMode.AUTON;
                state.shouldDeployIntake = false;
            }
        }
    }

    public void log() {
        Logger.recordOutput("RobotBrain/RobotState/opMode", state.opMode.name());
        Logger.recordOutput("RobotBrain/RobotState/isReal", state.isReal);
        Logger.recordOutput("RobotBrain/RobotState/isDSAttached", state.isDSAttached);
        Logger.recordOutput("RobotBrain/RobotState/isFMSAttached", state.isFMSAttached);
        Logger.recordOutput("RobotBrain/RobotState/isBrownedOut", state.isBrownedOut);

        Logger.recordOutput("RobotBrain/RobotState/isRed", state.isRed);
        Logger.recordOutput("RobotBrain/RobotState/autoWinnerIsKnown", state.autoWinnerIsKnown);
        Logger.recordOutput("RobotBrain/RobotState/didWinAuto", state.didWinAuto);

        Logger.recordOutput("RobotBrain/RobotState/modeTime", state.modeTime_s, Seconds.name());
        Logger.recordOutput("RobotBrain/RobotState/phase", state.phase.name());
        Logger.recordOutput("RobotBrain/RobotState/timeLeftInPhase", state.timeLeftInPhase_s, Seconds.name());
        Logger.recordOutput("RobotBrain/RobotState/fieldZone", state.fieldZone.name());
        Logger.recordOutput("RobotBrain/RobotState/isHubActive", state.isHubActive);

        Logger.recordOutput(
                "RobotBrain/RobotState/LauncherReport/turretOperational", state.launcherReport.turretOperational);
        Logger.recordOutput(
                "RobotBrain/RobotState/LauncherReport/turretIsAtTarget", state.launcherReport.turretIsAtTarget);
        Logger.recordOutput(
                "RobotBrain/RobotState/LauncherReport/shooterOperational", state.launcherReport.shooterOperational);
        Logger.recordOutput(
                "RobotBrain/RobotState/LauncherReport/shooterIsAtTarget", state.launcherReport.shooterIsAtTarget);
        Logger.recordOutput("RobotBrain/RobotState/isTurretHomed", state.isTurretHomed);
        Logger.recordOutput("RobotBrain/RobotState/SwerveReport/isOperational", state.swerveReport.isOperational);
        Logger.recordOutput(
                "RobotBrain/RobotState/IntakeReport/rollerOperational", state.intakeReport.rollerOperational);
        Logger.recordOutput(
                "RobotBrain/RobotState/IntakeReport/deployerOperational", state.intakeReport.deployerOperational);
        Logger.recordOutput("RobotBrain/RobotState/IntakeReport/hasDeployed", state.intakeReport.hasDeployed);

        Logger.recordOutput("RobotBrain/RobotState/targetingMode", state.targetingMode.name());
        Logger.recordOutput("RobotBrain/RobotState/overrideTurret", state.overrideTurret);
        Logger.recordOutput("RobotBrain/RobotState/ledsMode", state.ledsMode.name());
        Logger.recordOutput("RobotBrain/RobotState/driveMode", state.driveMode.name());
        Logger.recordOutput("RobotBrain/RobotState/shouldDeployIntake", state.shouldDeployIntake);
        Logger.recordOutput("RobotBrain/RobotState/driver1RumbleMode", state.driver1RumbleMode.name());
        Logger.recordOutput("RobotBrain/RobotState/driver2RumbleMode", state.driver2RumbleMode.name());

        if (state.isBrownedOut && !lastState.map(s -> s.isBrownedOut).orElse(false)) {
            Console.reportWarning("Brown out event triggered on RoboRIO", false);
        }
    }

    public void scheduleCommands() {
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

        if (lastState.isEmpty() || state.driveMode != lastState.get().driveMode) {
            switch (state.driveMode) {
                case DISABLED -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().swerve,
                        RobotContainer.instance().stopSwerveCmd());
                case AUTON -> {
                    changeSubsystemDefaultCommand(
                            RobotContainer.instance().swerve,
                            RobotContainer.instance().brakeSwerveCmd());
                    CommandScheduler.getInstance()
                            .schedule(RobotContainer.instance().autonCmd());
                }
                case TELEOP -> changeSubsystemDefaultCommand(
                        RobotContainer.instance().swerve,
                        RobotContainer.instance().teleopDriveCmd());
            }
        }

        if (state.shouldDeployIntake) {
            CommandScheduler.getInstance()
                    .schedule(RobotContainer.instance()
                            .deployIntakeCmd()
                            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
        }

        if (lastState.isEmpty() || state.driver1RumbleMode != lastState.get().driver1RumbleMode) {
            updateRumbleMode(state.driver1RumbleMode, RobotContainer.instance().driver1);
        }
        if (lastState.isEmpty() || state.driver2RumbleMode != lastState.get().driver2RumbleMode) {
            updateRumbleMode(state.driver2RumbleMode, RobotContainer.instance().driver2);
        }
    }

    private void updateRumbleMode(RumbleMode newMode, Controller<?, ?> contr) {
        switch (newMode) {
            case IDLE -> removeSubsystemDefaultCommand(contr);
            case CONTINUOUS -> changeSubsystemDefaultCommand(contr, contr.rumbleCmd(1));
            case KNOCK -> {
                removeSubsystemDefaultCommand(contr);
                CommandScheduler.getInstance().schedule(contr.knockCmd(0.75));
            }
            case NORMAL_PULSING -> changeSubsystemDefaultCommand(contr, contr.pulseRumbleCmd(0.25, 0.25, 1));
            case FAST_PULSING -> changeSubsystemDefaultCommand(contr, contr.pulseRumbleCmd(0.17, 0.08, 1));
        }
    }

    public boolean isInFZone() {
        return state.fieldZone == FieldZone.BLUE && !state.isRed || state.fieldZone == FieldZone.RED && state.isRed;
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

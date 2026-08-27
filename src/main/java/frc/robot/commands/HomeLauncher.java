package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.LauncherConfig.TurretConfig;
import frc.robot.subsystems.launcher.Launcher;
import org.littletonrobotics.junction.Logger;

public class HomeLauncher extends Command {

    private final Launcher launcher;
    private Timer startTimer = new Timer();

    public HomeLauncher(Launcher _launcher) {
        launcher = _launcher;
        addRequirements(_launcher);
    }

    @Override
    public void initialize() {
        launcher.setTurretVoltage(TurretConfig.homingVoltage.in(Volts));
        startTimer.restart();
    }

    @Override
    public void execute() {
        Logger.recordOutput("HomeLauncher/timeSinceStart", startTimer.get(), Seconds.name());
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopTurret();
        Logger.recordOutput("HomeLauncher/timeSinceStart", Double.NaN, Seconds.name());

        if (!interrupted) {
            launcher.setAsTurretHomingPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return startTimer.hasElapsed(TurretConfig.homingMinRunTime) && launcher.isTurretAtHomingLimit()
                || startTimer.hasElapsed(TurretConfig.homingTimeout);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}

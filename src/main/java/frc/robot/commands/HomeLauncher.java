package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.LauncherConfig.TurretConfig;
import frc.robot.subsystems.launcher.Launcher;

public class HomeLauncher extends Command {
    private final Launcher launcher;
    private Timer startTimer = new Timer();

    public HomeLauncher(Launcher _launcher) {
        launcher = _launcher;
        addRequirements(_launcher);
    }

    @Override
    public void initialize() {
        launcher.setTurretDuty(TurretConfig.calibrationSpeed);
        startTimer.restart();
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopTurret();

        if (!interrupted) {
            launcher.setAsTurretHomingPosition();
        }
    }

    @Override
    public boolean isFinished() {
        return startTimer.hasElapsed(TurretConfig.calibrationEndDelay) && launcher.isTurretAtHomingLimit()
                || startTimer.hasElapsed(TurretConfig.calibrationTimeout);
    }

    @Override
    public InterruptionBehavior getInterruptionBehavior() {
        return InterruptionBehavior.kCancelIncoming;
    }
}

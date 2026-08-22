package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.config.LauncherConfig.TurretConfig;
import frc.robot.subsystems.launcher.Launcher;
import frc.robot.telemetry.Telemetry;
import frc.robot.telemetry.TelemetryUnits;
import frc.robot.telemetry.writer.DoubleWriter;

public class HomeLauncher extends Command {

    private static final DoubleWriter timeSinceStartWriter = Telemetry.makeDoubleWriterInitial(
            HomeLauncher.class.getSimpleName(), "timeSinceStart", TelemetryUnits.seconds, Double.NaN);

    private final Launcher launcher;
    private Timer startTimer = new Timer();

    public HomeLauncher(Launcher _launcher) {
        launcher = _launcher;
        addRequirements(_launcher);
    }

    @Override
    public void initialize() {
        launcher.setTurretDuty(TurretConfig.homingSpeed);
        startTimer.restart();
    }

    @Override
    public void execute() {
        timeSinceStartWriter.set(startTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        launcher.stopTurret();
        timeSinceStartWriter.set(Double.NaN);

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

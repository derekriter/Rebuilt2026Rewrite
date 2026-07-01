package frc.robot.subsystems.launcher.shooter;

public record ShooterFlags(
        boolean motorConnected,
        boolean motorOverheating,
        boolean motorCriticalOverheating,
        boolean motorConfigSuccessfull,
        boolean motorHardwareFaultActive,
        boolean overheatShutdown) {}

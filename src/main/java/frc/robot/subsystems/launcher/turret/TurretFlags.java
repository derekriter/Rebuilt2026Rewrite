package frc.robot.subsystems.launcher.turret;

public record TurretFlags(
        boolean motorConnected,
        boolean motorOverheating,
        boolean motorCriticalOverheating,
        boolean motorConfigSuccessfull,
        boolean motorHardwareFaultActive,
        boolean overheatShutdown) {}

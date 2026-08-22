package frc.robot.subsystems.launcher;

public final class LauncherReport {

    public boolean turretOperational = true;

    public boolean shooterOperational = true;
    public boolean shooterIsAtTarget = false;

    public void copyFrom(LauncherReport ref) {
        turretOperational = ref.turretOperational;

        shooterOperational = ref.shooterOperational;
        shooterIsAtTarget = ref.shooterIsAtTarget;
    }
}

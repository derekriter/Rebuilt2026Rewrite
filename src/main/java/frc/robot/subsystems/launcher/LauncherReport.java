package frc.robot.subsystems.launcher;

public final class LauncherReport {

    public boolean turretOperational = true;
    public boolean turretIsAtTarget = false;

    public boolean shooterOperational = true;
    public boolean shooterIsAtTarget = false;

    public void copyFrom(LauncherReport ref) {
        turretOperational = ref.turretOperational;
        turretIsAtTarget = ref.turretIsAtTarget;

        shooterOperational = ref.shooterOperational;
        shooterIsAtTarget = ref.shooterIsAtTarget;
    }
}

package frc.robot.subsystems.launcher;

public final class LauncherReport {

    public boolean turretOperational = false;
    public boolean turretIsAtTarget = false;

    public boolean shooterOperational = false;
    public boolean shooterIsAtTarget = false;

    public void copyFrom(LauncherReport ref) {
        turretOperational = ref.turretOperational;
        turretIsAtTarget = ref.turretIsAtTarget;

        shooterOperational = ref.shooterOperational;
        shooterIsAtTarget = ref.shooterIsAtTarget;
    }
}

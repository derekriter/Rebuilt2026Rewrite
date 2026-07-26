package frc.robot.subsystems.launcher;

public class LauncherFlags {

    public boolean turretOperational = true;

    public boolean shooterOperational = true;
    public boolean shooterIsAtTarget = false;

    public void copyFrom(LauncherFlags ref) {
        turretOperational = ref.turretOperational;

        shooterOperational = ref.shooterOperational;
        shooterIsAtTarget = ref.shooterIsAtTarget;
    }
}
